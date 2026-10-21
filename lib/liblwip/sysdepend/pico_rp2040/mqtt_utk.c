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
/* Robot telemetry / command bridge (app_program/subsystem1, Buddy 1).
 * Plain-C header: safe to include here. */
#include "../../../../app_program/subsystem1/s1_bridge.h"

#define MQTT_UTK_HEARTBEAT_MS   5000
/* lwIP pings at half this interval and closes the connection (status
 * MQTT_CONNECT_TIMEOUT -> connection_cb) when the broker stays silent, so a
 * dead broker or link is noticed within about this time. */
#define MQTT_UTK_KEEPALIVE_S    15
#define MQTT_UTK_RETRY_MS       3000
#define MQTT_UTK_TELEMETRY_MS   100	/* poll the S1 queue this often; S1 queues a line every 200 ms */
#define MQTT_UTK_TELEMETRY_MAX  4	/* messages per drain */

/* Everything lives under S1_TOPIC_ROOT.  "status" is retained: "online" when
 * connected, "offline" published by the broker as the last will. */
#define MQTT_UTK_TOPIC_HEARTBEAT	S1_TOPIC_ROOT "heartbeat"
#define MQTT_UTK_TOPIC_STATUS		S1_TOPIC_ROOT "status"

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks, so the topic has to be kept until the fragment
 * carrying MQTT_DATA_FLAG_LAST shows up. Oversized commands are rejected
 * rather than being allowed to run off the end of either buffer. */
#define MQTT_UTK_RX_TOPIC_MAX   64
#define MQTT_UTK_RX_DATA_MAX    128

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
	rx_topic_truncated = (topic[i] != '\0');
	rx_truncated = (tot_len >= MQTT_UTK_RX_DATA_MAX);
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;

	for(i = 0; i < len; i++) {
		if(rx_len < (MQTT_UTK_RX_DATA_MAX - 1) &&
		   rx_len < rx_expected_len) {
			rx_data[rx_len++] = data[i];
		} else {
			rx_truncated = 1;
		}
	}

	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
		rx_data[rx_len] = '\0';
		tm_printf((UB*)"mqtt: RX [%s] \"%s\"%s\n", rx_topic, rx_data,
			  rx_truncated? " (truncated)": "");
		if(topic_is(rx_topic, S1_TOPIC_CMD)) {
			s1_bridge_on_command((const char *)rx_data, (int)rx_len);
		}
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed to " S1_TOPIC_CMD "\n");
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

static void heartbeat_timeout(void *arg)
{
	static unsigned int	count = 0;
	UB			payload[48];
	int			len;
	err_t			err;

	(void)arg;

	if(mqtt_client_is_connected(client)) {
		len = tm_sprintf(payload,
				 (const UB*)"{\"status\":\"online\",\"seq\":%u}",
				 count++);
		if(len > 0 && len < sizeof(payload)) {
			err = mqtt_publish(client, MQTT_UTK_TOPIC_HEARTBEAT, payload,
					   (u16_t)len, 0 /* qos */, 0 /* retain */,
					   publish_done_cb, NULL);
			if(err != ERR_OK) {
				tm_printf((UB*)"mqtt: heartbeat enqueue failed, err=%d\n",
					  (int)err);
			}
		} else {
			tm_printf((UB*)"mqtt: heartbeat formatting failed\n");
		}
	}

	/* Re-arms itself every call -- mqtt.c's connection callback (below)
	 * only starts this chain once, on the transition into MQTT_CONNECT_
	 * ACCEPTED, so there is exactly one heartbeat chain alive at a time. */
	sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

/* Publish what the application queued (s1_post), a few per call so this
 * task keeps servicing the radio. Re-arms itself like the heartbeat. */
static void telemetry_timeout(void *arg)
{
	static char	topic[S1_BRIDGE_TOPIC_MAX];
	static char	payload[S1_BRIDGE_LINE_MAX];
	int		i, len;

	(void)arg;

	for(i = 0; i < MQTT_UTK_TELEMETRY_MAX && mqtt_client_is_connected(client); i++) {
		len = s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload));
		if(len <= 0) {
			break;
		}
		(void)mqtt_publish(client, topic, payload, (u16_t)len,
				   0 /* qos */, 0 /* retain */,
				   publish_done_cb, NULL);
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

		/* Register receive callbacks before subscribing. Only the exact
		 * payloads accepted by s1_bridge_on_command() have any effect. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		err = mqtt_subscribe(client, S1_TOPIC_CMD, 0 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: subscribe " S1_TOPIC_CMD " failed, err=%d\n",
				  (int)err);
		}

		/* Also drops telemetry queued while offline, so a reconnect
		 * never publishes obsolete vehicle state. */
		s1_bridge_set_connected(1);

		(void)mqtt_publish(client, MQTT_UTK_TOPIC_STATUS, "online", 6,
				   0 /* qos */, 1 /* retain */,
				   publish_done_cb, NULL);

		/* Keep exactly one timer chain for each periodic publisher. */
		sys_untimeout(heartbeat_timeout, NULL);
		sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
		sys_untimeout(telemetry_timeout, NULL);
		sys_timeout(MQTT_UTK_TELEMETRY_MS, telemetry_timeout, NULL);
	} else {
		s1_bridge_set_connected(0);
		/* mqtt_client_connect() reconnects on its own for a dropped
		 * keepalive; a rejected/never-established connection does not
		 * retry itself, so do that here -- on a delay, so a broker
		 * that keeps rejecting (e.g. bad client ID) can't turn this
		 * into a tight reconnect loop. */
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, MQTT_UTK_RETRY_MS / 1000);
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
	client_info.will_topic = MQTT_UTK_TOPIC_STATUS;
	client_info.will_msg = "offline";
	client_info.will_qos = 0;
	client_info.will_retain = 1;

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;
	}
}
