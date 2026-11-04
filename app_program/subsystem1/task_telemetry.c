/*
 *      task_telemetry.c
 *      Subsystem 1 -- telemetry_task, every 200 ms.
 *
 *      Packs a versioned 14-byte frame for MQTT (topic robot/g10/t).
 *      A readable JSON snapshot is still printed once a second to USB while
 *      MQTT is disconnected.
 *
 *      The field layout and scales are defined in s1_codec.h and TEAM.md.
 */

#include "subsystem1.h"
#include "s1_codec.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"
#include "../vehicle_controller/vehicle_controller.h"
#include "../vehicle_controller/vc_sm.h"
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
    S1_TLM           tlm;
    uint8_t          frame[S1_TLM_BYTES];
    uint8_t          buf[S1_CONSOLE_LINE_MAX]; /* tm_sprintf has no bound */
    uint32_t         tick = 0;
    uint8_t          seq = 0;
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
        tlm.version      = S1_PROTO_VERSION;
        tlm.seq          = seq++;
        tlm.state        = (uint8_t)vc_st.state;
        tlm.speed_l_mms  = motion_st.speed_l_mm_s;
        tlm.speed_r_mms  = motion_st.speed_r_mm_s;
        tlm.distance_mm  = motion_st.distance_mm;
        tlm.line_pos     = line_st.position;
        tlm.obs_dist_cm  = (obs_st.last_distance_mm == 0) ?
                           S1_OBS_MAX_CM :
                           (uint32_t)(obs_st.last_distance_mm / 10);
        tlm.obs_detected = (obs_st.state >= S5_ST_DETECTED) &&
                           (obs_st.state != S5_ST_ERROR);
        tlm.obs_state    = (uint8_t)obs_st.state;
        tlm.nav          = (uint8_t)vc_st.last_nav;
        tlm.barcodes     = (vc_st.barcodes_read > 15U) ?
                           15U : (uint8_t)vc_st.barcodes_read;
        tlm.pitch_cdeg   = imu_st.pitch_cdeg;
        tlm.hump_now_um  = imu_st.hump_height_um;
        tlm.hump_max_um  = imu_st.max_peak_um;
        tlm.fault_code   = vc_st.last_fault_code;
        tlm.wifi_up      = g_s1_stat.wifi_up;
        s1_tlm_encode(&tlm, frame);
        (void)s1_post(S1_TOPIC_TLM, frame, sizeof(frame));

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

        if ((!g_s1_stat.mqtt_up) && (0 == (tick % S1_CONSOLE_EVERY)))
        {
            tm_printf((uint8_t *)"%s\n", buf);
        }
    }
}

/*** end of file ***/
