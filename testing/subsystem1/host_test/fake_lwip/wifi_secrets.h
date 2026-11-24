/* Host stand-in for config/wifi_secrets.h (the owner's build overrides the
   heartbeat topic like this). */
#ifndef FAKE_WIFI_SECRETS_H
#define FAKE_WIFI_SECRETS_H
#define MQTT_BROKER_IP   "172.20.10.3"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "host-test"
#define MQTT_TOPIC       "mtk3smp-pico/heartbeat"
#endif
