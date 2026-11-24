/* Host stand-in for lwip/ip_addr.h (types are in the fake mqtt.h). */
#ifndef FAKE_LWIP_IP_ADDR_H
#define FAKE_LWIP_IP_ADDR_H
#include "lwip/apps/mqtt.h"
#define IPADDR_TYPE_V4 0
#define ip_2_ip4(ipaddr) (&((ipaddr)->ip4))
#define IP_SET_TYPE(ipaddr, iptype) ((ipaddr)->type = (iptype))
int ip4addr_aton(const char * cp, ip4_addr_t * addr);
#endif
