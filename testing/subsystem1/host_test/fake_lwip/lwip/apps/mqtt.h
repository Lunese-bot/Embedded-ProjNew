/* Host stand-in for lwIP's lwip/apps/mqtt.h: only what mqtt_utk.c uses.
   The behaviour lives in test_mqtt_utk.c. */
#ifndef FAKE_LWIP_MQTT_H
#define FAKE_LWIP_MQTT_H

#include <stdint.h>
#include <stddef.h>

/* the project's real lwIP options, then lwIP's own defaults (mqtt_opts.h) */
#include "../../../../../../lib/liblwip/sysdepend/pico_rp2040/lwipopts.h"
#ifndef MQTT_OUTPUT_RINGBUF_SIZE
#define MQTT_OUTPUT_RINGBUF_SIZE 256
#endif
#ifndef MQTT_REQ_MAX_IN_FLIGHT
#define MQTT_REQ_MAX_IN_FLIGHT 4
#endif

typedef uint8_t  u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int8_t   err_t;

#define ERR_OK      0
#define ERR_MEM     (-1)
#define ERR_TIMEOUT (-3)
#define ERR_CONN    (-11)

#define MQTT_DATA_FLAG_LAST 1

typedef enum
{
    MQTT_CONNECT_ACCEPTED     = 0,
    MQTT_CONNECT_DISCONNECTED = 256,
    MQTT_CONNECT_TIMEOUT      = 257
} mqtt_connection_status_t;

typedef struct mqtt_client_s mqtt_client_t;
typedef struct { u32_t addr; } ip4_addr_t;
typedef struct { ip4_addr_t ip4; int type; } ip_addr_t;

struct mqtt_connect_client_info_t
{
    const char * client_id;
    const char * client_user;
    const char * client_pass;
    u16_t        keep_alive;
};

typedef void (*mqtt_request_cb_t)(void * arg, err_t err);
typedef void (*mqtt_connection_cb_t)(mqtt_client_t * client, void * arg,
                                     mqtt_connection_status_t status);
typedef void (*mqtt_incoming_publish_cb_t)(void * arg, const char * topic,
                                           u32_t tot_len);
typedef void (*mqtt_incoming_data_cb_t)(void * arg, const u8_t * data,
                                        u16_t len, u8_t flags);

mqtt_client_t * mqtt_client_new(void);
err_t mqtt_client_connect(mqtt_client_t * client, const ip_addr_t * ip,
                          u16_t port, mqtt_connection_cb_t cb, void * arg,
                          const struct mqtt_connect_client_info_t * info);
u8_t  mqtt_client_is_connected(mqtt_client_t * client);
void  mqtt_set_inpub_callback(mqtt_client_t * client,
                              mqtt_incoming_publish_cb_t pub_cb,
                              mqtt_incoming_data_cb_t data_cb, void * arg);
err_t mqtt_subscribe(mqtt_client_t * client, const char * topic, u8_t qos,
                     mqtt_request_cb_t cb, void * arg);
err_t mqtt_publish(mqtt_client_t * client, const char * topic,
                   const void * payload, u16_t payload_length, u8_t qos,
                   u8_t retain, mqtt_request_cb_t cb, void * arg);

#endif
