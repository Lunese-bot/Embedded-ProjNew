/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *	time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *	timers, and the heartbeat below, both ride the same sys_check_timeouts()
 *	polling that task already does for DHCP and TCP -- nothing extra runs
 *	on any other task or core.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>
#ifndef MQTT_TOPIC
#define MQTT_TOPIC "robot/g10/heartbeat"
#endif
/* Robot telemetry / command bridge (app_program/subsystem1, Buddy 1).
 * Plain-C header: safe to include here. */
#include "../../../../app_program/subsystem1/s1_bridge.h"
#include "../../../../app_program/subsystem1/s1_stats.h"

#define MQTT_UTK_HEARTBEAT_MS   5000
#define MQTT_UTK_KEEPALIVE_S    60
#define MQTT_UTK_RETRY_MS       3000
#define MQTT_UTK_TELEMETRY_MS   100	/* drain the S1 queue this often */
#define MQTT_UTK_TELEMETRY_MAX  4	/* publish attempts per drain */

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks, so the topic has to be kept until the fragment
 * carrying MQTT_DATA_FLAG_LAST shows up. Both buffers are sized well past the
 * heartbeat this demo publishes; anything longer is truncated and flagged
 * rather than being allowed to run off the end. */
#define MQTT_UTK_RX_TOPIC_MAX   64
#define MQTT_UTK_RX_DATA_MAX    320	/* holds the longest heartbeat echo */

/* Heartbeat text: "hb <count> " then the S1 delivery counters. */
#define MQTT_UTK_HB_PREFIX_MAX  14	/* "hb 4294967295 " */
#define MQTT_UTK_HB_MAX         (MQTT_UTK_HB_PREFIX_MAX + S1_STATS_TEXT_MAX)

/* Compile-time checks (C89-safe): a negative array size fails the build. */
typedef char mqtt_utk_hb_echo_fits[(MQTT_UTK_HB_MAX <= MQTT_UTK_RX_DATA_MAX) ? 1 : -1];
typedef char mqtt_utk_hb_len_fits[(MQTT_UTK_HB_MAX <= 0xFFFF) ? 1 : -1];

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;

static char		rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB		rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t		rx_len;
static u32_t		rx_expected_len;
static u8_t		rx_topic_truncated;
static u8_t		rx_truncated;

static void heartbeat_timeout(void *arg);
static void telemetry_timeout(void *arg);

static int topic_is(const char *a, const char *b)
{
	while(*a != '\0' && *a == *b) {
		a++;
		b++;
	}
	return *a == *b;
}

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- copy it out. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	u32_t	i;

	(void)arg;
	for(i = 0; i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
		rx_topic[i] = topic[i];
	}
	rx_topic[i] = '\0';

	rx_len = 0;
	rx_expected_len = tot_len;
	rx_topic_truncated = topic[i] != '\0';
	rx_truncated = tot_len > sizeof(rx_data);
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;

	for(i = 0; i < len; i++) {
		if(rx_len < sizeof(rx_data) && rx_len < rx_expected_len) {
			rx_data[rx_len++] = data[i];
		} else {
			rx_truncated = 1;
		}
	}

	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
		tm_printf((UB*)"mqtt: RX [%s] %d byte(s)%s\n", rx_topic,
			  (int)rx_len, rx_truncated? " (truncated)": "");
		if(!rx_topic_truncated && topic_is(rx_topic, S1_TOPIC_CMD)) {
			s1_bridge_on_command((const uint8_t *)rx_data,
					     (rx_truncated || rx_len != rx_expected_len)
					     ? -1 : (int32_t)rx_len);
		}
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscription accepted\n");
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		tm_printf((UB*)"mqtt: publish failed, err=%d\n", (int)result);
	}
}

/* Completion of an S1 publish; arg is its s1_bridge_publish_tag(). */
static void s1_publish_done_cb(void *arg, err_t result)
{
	s1_bridge_publish_done(arg, (int32_t)result);
	publish_done_cb(NULL, result);
}

/* "hb <count> s1 tq=..." into payload[size]; bounded, returns length. */
static int heartbeat_text(char *payload, int size, unsigned int count)
{
	char		digits[10];
	int		n = 0, len = 0;

	if(size < MQTT_UTK_HB_PREFIX_MAX + 1) {
		return 0;
	}
	payload[len++] = 'h';
	payload[len++] = 'b';
	payload[len++] = ' ';
	do {
		digits[n++] = (char)('0' + (count % 10U));
		count /= 10U;
	} while(count != 0U && n < (int)sizeof(digits));
	while(n > 0) {
		payload[len++] = digits[--n];
	}
	payload[len++] = ' ';
	len += (int)s1_bridge_format_counters(&payload[len], (int32_t)(size - len));
	return len;
}

static void heartbeat_timeout(void *arg)
{
	static unsigned int	count = 0;
	static char		payload[MQTT_UTK_HB_MAX];
	int			len;
	err_t			err;

	(void)arg;

	if(mqtt_client_is_connected(client)) {
		len = heartbeat_text(payload, (int)sizeof(payload), count++);
		tm_printf((UB*)"mqtt: TX [" MQTT_TOPIC "] \"%s\"\n", payload);
		err = mqtt_publish(client, MQTT_TOPIC, payload, (u16_t)len,
				   0 /* qos */, 0 /* retain */,
				   publish_done_cb, NULL);
		s1_bridge_heartbeat_result((int32_t)err);
	}

	/* Re-arms itself every call -- mqtt.c's connection callback (below)
	 * only starts this chain once, on the transition into MQTT_CONNECT_
	 * ACCEPTED, so there is exactly one heartbeat chain alive at a time. */
	sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

/* S1's publish adapter: the only S1 call into mqtt_publish().  The topic
 * and payload stay valid for the call; lwIP copies them into its ring. */
static int32_t s1_publish(const char *topic, const uint8_t *payload,
			  uint16_t len, void *tag)
{
	if(!mqtt_client_is_connected(client)) {
		return ERR_CONN;
	}
	return (int32_t)mqtt_publish(client, topic, (const u8_t *)payload,
				     (u16_t)len, 0 /* qos */, 0 /* retain */,
				     s1_publish_done_cb, tag);
}

/* Publish what the application queued (s1_post), a few per call so this
 * task keeps servicing the radio. S1 decides the order and keeps every
 * message until lwIP accepts it (s1_txq.h). Re-arms itself like the
 * heartbeat. */
static void telemetry_timeout(void *arg)
{
	(void)arg;

	if(mqtt_client_is_connected(client)) {
		(void)s1_bridge_service(s1_publish, MQTT_UTK_TELEMETRY_MAX);
	}
	sys_timeout(MQTT_UTK_TELEMETRY_MS, telemetry_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		err_t	err;

		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;

		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Subscribe to the same topic this client publishes to, so every
		 * heartbeat comes back from the broker and is printed by
		 * incoming_data_cb(). A matching RX for each TX is end-to-end
		 * proof of the round trip without needing mosquitto_sub on the
		 * broker host. Callbacks must be registered before subscribing,
		 * or the first delivery has nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		err = mqtt_subscribe(client, MQTT_TOPIC, 0 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}
		/* robot remote commands (subsystem 1) */
		err = mqtt_subscribe(client, S1_TOPIC_CMD, 0 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: subscribe " S1_TOPIC_CMD " failed, err=%d\n",
				  (int)err);
		}
		s1_bridge_set_connected(1);
		sys_untimeout(telemetry_timeout, NULL);
		sys_timeout(MQTT_UTK_TELEMETRY_MS, telemetry_timeout, NULL);

		/* Drop any chain left over from a previous connection before
		 * arming a new one. Each reconnect would otherwise add another
		 * self-perpetuating heartbeat, and every one of them holds a
		 * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
		sys_untimeout(heartbeat_timeout, NULL);
		sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
	} else {
		s1_bridge_set_connected(0);
		/* mqtt_client_connect() reconnects on its own for a dropped
		 * keepalive; a rejected/never-established connection does not
		 * retry itself, so do that here -- on a delay, so a broker
		 * that keeps rejecting (e.g. bad client ID) can't turn this
		 * into a tight reconnect loop. */
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, MQTT_UTK_RETRY_MS / 1000);
		/* one retry chain at most, however many failures arrive */
		sys_untimeout(retry_timeout, NULL);
		sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;
	}
}
