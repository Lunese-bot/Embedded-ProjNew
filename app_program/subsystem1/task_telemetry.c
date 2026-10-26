/*
 *      task_telemetry.c
 *      Subsystem 1 -- telemetry_task, every 200 ms.
 *
 *      Builds one JSON line from every subsystem's status and queues it for
 *      MQTT (topic robot/g10/telemetry).  Without a WiFi build (or before
 *      MQTT connects) the line is also printed once a second, so the same
 *      telemetry can be checked on the console.
 *
 *      TODO(Buddy 1): agree the topic list and JSON fields with the team
 *      and record them in TEAM.md section 5 / the MQTT documentation.
 */

#include "subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem6/subsystem6.h"
#include "../subsystem6/vc_sm.h"
#if TM_WIFI_CYW43
#include "cyw43_utk.h"

#define S1_CYW43_LINK_UP 3 /* CYW43_LINK_UP in the Pico SDK's cyw43.h */
#endif

#define S1_UM_PER_MM 1000

void telemetry_task(int32_t stacd, void * p_exinf)
{
    vc_status_t      vc_st;
    motion_status_t  motion_st;
    s3_line_status_t line_st;
    imu_status_t     imu_st;
    s5_status_t      obs_st;
    uint8_t          buf[S1_LINE_MAX + 32]; /* tm_sprintf has no bound */
    uint32_t         tick = 0;
    SYSTIM           now;
#if TM_WIFI_CYW43
    T_CYW43_UTK_STATUS wifi_st;
#endif

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        tk_slp_tsk(TMO_FEVR); /* every 200 ms */
        tick++;

        vc_get_status(&vc_st);
        s2_get_status(&motion_st);
        s3_get_line_status(&line_st);
        s4_get_status(&imu_st);
        s5_get_status(&obs_st);
        tk_get_tim(&now);

#if TM_WIFI_CYW43
        cyw43_utk_get_status(&wifi_st);
        g_s1_stat.wifi_up = (S1_CYW43_LINK_UP == wifi_st.link_status);
#endif
        /* The casts match each value to its %d / %u conversion.  Every
           value fits: they are 32-bit or smaller on this target. */
        tm_sprintf(
            buf,
            (const uint8_t *)"{\"t\":%u,\"st\":\"%s\",\"v\":[%d,%d],\"odo\":%d,"
                             "\"line\":%d,\"bc\":%u,\"nav\":%d,\"pitch\":%d,"
                             "\"hump\":%d,\"maxhump\":%d,\"us\":%d,\"obs\":%d}",
            (unsigned)now.lo, vc_state_name(vc_st.state),
            (int)motion_st.speed_l_mm_s, (int)motion_st.speed_r_mm_s,
            (int)motion_st.distance_mm, (int)line_st.position,
            (unsigned)vc_st.barcodes_read, (int)vc_st.last_nav,
            (int)imu_st.pitch_cdeg, (int)(imu_st.hump_height_um / S1_UM_PER_MM),
            (int)(imu_st.max_peak_um / S1_UM_PER_MM),
            (int)obs_st.last_distance_mm, (int)obs_st.state);

        s1_post("telemetry", (const char *)buf);

        if ((!g_s1_stat.mqtt_up) && (0 == (tick % S1_CONSOLE_EVERY)))
        {
            tm_printf((uint8_t *)"%s\n", buf);
        }
    }
}

/*** end of file ***/
