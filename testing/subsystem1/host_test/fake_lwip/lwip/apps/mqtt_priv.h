/* Host stand-in for lwIP's lwip/apps/mqtt_priv.h: the client fields that
   mqtt_utk.c reads (connection flag for the fake, output ring indexes). */
#ifndef FAKE_LWIP_MQTT_PRIV_H
#define FAKE_LWIP_MQTT_PRIV_H
#include "lwip/apps/mqtt.h"

struct mqtt_ringbuf_t
{
    u16_t put;
    u16_t get;
};

struct mqtt_client_s
{
    int32_t               connected; /* fake only */
    struct mqtt_ringbuf_t output;
};
#endif
