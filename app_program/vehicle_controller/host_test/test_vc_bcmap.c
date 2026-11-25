/*
 *      test_vc_bcmap.c
 *      PC tests for the dashboard-configurable barcode map:
 *        - wire codes, 2-bit packing (golden bytes), all 26 letters x 4 actions
 *        - storage records, A/B slots, interrupted and failed writes, reboot
 *        - the binary MQTT protocol (subsystem1/s1_cfg.c)
 *        - the Vehicle Controller entry points: VC_EV_CMD_CONFIG decides
 *          apply / refuse from its own state, a decoded letter resolves
 *          against the active map and executes once at the next junction.
 *      No Pico, no kernel.  From this folder:  make
 */

#include <stdio.h>
#include <string.h>
#include "../vc_sm.h"
#include "../vc_barcode_rules.h"
#include "../vc_barcode_map.h"
#include "../../subsystem1/s1_cfg.h"

static int32_t g_passed;
static int32_t g_failed;

#define CHECK(id, cond, desc)                                            \
    do                                                                   \
    {                                                                    \
        if (cond)                                                        \
        {                                                                \
            g_passed++;                                                  \
            printf("[TEST] %s PASS  %s\n", id, desc);                    \
        }                                                                \
        else                                                             \
        {                                                                \
            g_failed++;                                                  \
            printf("[TEST] %s FAIL  %s   (%s:%d)\n", id, desc, __FILE__, \
                   __LINE__);                                            \
        }                                                                \
    } while (0)

/* ---- fake flash: two 256-byte slots that behave like erase + program --- */

typedef enum
{
    FAIL_NONE = 0,
    FAIL_BEFORE_ERASE,  /* refused (other core did not park): untouched */
    FAIL_AFTER_ERASE,   /* power cut after the erase */
    FAIL_HALF_PROGRAM,  /* power cut half way through programming */
    FAIL_SILENT_CORRUPT /* reports success, one bit wrong: read-back catches it
                         */
} fail_t;

typedef struct
{
    uint8_t  slot[VC_BCMAP_SLOTS][256];
    uint32_t writes;
    uint32_t writes_to[VC_BCMAP_SLOTS];
    fail_t   fail_next;
    int      read_error;
} fake_flash_t;

static fake_flash_t g_flash;

static int32_t fake_read(void * p_ctx, uint32_t slot, uint8_t * p_buf,
                         uint32_t len)
{
    fake_flash_t * p_val = (fake_flash_t *)p_ctx;

    if ((slot >= VC_BCMAP_SLOTS) || (len > 256u) || p_val->read_error)
    {
        return -1;
    }
    memcpy(p_buf, p_val->slot[slot], len);
    return 0;
}

static int32_t fake_write(void * p_ctx, uint32_t slot, const uint8_t * p_buf,
                          uint32_t len)
{
    fake_flash_t * p_val = (fake_flash_t *)p_ctx;
    fail_t         fail  = p_val->fail_next;

    p_val->fail_next = FAIL_NONE;
    if ((slot >= VC_BCMAP_SLOTS) || (len > 256u))
    {
        return -1;
    }
    if (FAIL_BEFORE_ERASE == fail)
    {
        return -1;
    }
    p_val->writes++;
    p_val->writes_to[slot]++;
    memset(p_val->slot[slot], 0xFF, sizeof(p_val->slot[slot]));
    if (FAIL_AFTER_ERASE == fail)
    {
        return -1;
    }
    if (FAIL_HALF_PROGRAM == fail)
    {
        memcpy(p_val->slot[slot], p_buf, len / 2u);
        return -1;
    }
    memcpy(p_val->slot[slot], p_buf, len);
    if (FAIL_SILENT_CORRUPT == fail)
    {
        p_val->slot[slot][13] ^= 0x04u; /* one map bit flipped */
    }
    return 0;
}

static const vc_bcmap_store_t g_store = { fake_read, fake_write, &g_flash };

static void flash_blank(void)
{
    memset(&g_flash, 0, sizeof(g_flash));
    memset(g_flash.slot, 0xFF, sizeof(g_flash.slot));
}

/* power-up: load whatever the slots hold */
static void reboot(void)
{
    vc_bcmap_init(&g_store);
}

/* ---- helpers -------------------------------------------------------- */

static const uint8_t FACTORY_GOLDEN[7] = { 0x00, 0x01, 0x00, 0x00,
                                           0x00, 0x00, 0x20 };

static void map_set(uint8_t p_packed[7], char letter, uint32_t code)
{
    uint32_t idx2  = (uint32_t)(letter - 'A');
    uint32_t shift = 6u - 2u * (idx2 % 4u);

    p_packed[idx2 / 4u] =
        (uint8_t)((p_packed[idx2 / 4u] & ~(3u << shift)) | (code << shift));
}

static vc_bcmap_snapshot_t snap(void)
{
    vc_bcmap_snapshot_t s_val;

    memset(&s_val, 0, sizeof(s_val));
    (void)vc_bcmap_get(&s_val);
    return s_val;
}

static int all_letters_match(const uint8_t p_packed[7])
{
    char text[2] = { 'A', '\0' };

    for (uint32_t idx2 = 0u; idx2 < VC_BCMAP_LETTERS; idx2++)
    {
        text[0] = (char)('A' + (int)idx2);
        if (vc_bcmap_nav_from_wire(vc_bcmap_packed_code(p_packed, idx2)) !=
            vc_barcode_lookup(text))
        {
            return 0;
        }
    }
    return 1;
}

/* ---- the Vehicle Controller, driven as the task does ----------------- */

static vc_sm_t      g_sm;
static vc_actions_t g_actions;
static uint32_t     g_now_ms;
static int32_t      g_tally[32];

/* VC_ACT_JUNCTION notes seen since the last tally_clear() */
static int32_t g_jx_n;
static int32_t g_jx[16][3]; /* kind, letter, code */

static void tally_clear(void)
{
    memset(g_tally, 0, sizeof(g_tally));
    g_jx_n = 0;
}

static int jx_is(int32_t idx2, int32_t kind, int32_t letter, int32_t code)
{
    return (idx2 < g_jx_n) && (g_jx[idx2][0] == kind) &&
           (g_jx[idx2][1] == letter) && (g_jx[idx2][2] == code);
}

/* the reply the task would publish for the last config action */
static int      g_reply_valid;
static uint16_t g_reply_id;
static uint8_t  g_reply[S1_CFG_REPLY_LEN];

static void reply_now(uint8_t opcode, uint16_t obj_id, uint8_t result)
{
    vc_bcmap_snapshot_t s_val = snap();

    (void)s1_cfg_encode_reply(opcode, obj_id, result, (uint16_t)s_val.gen,
                              s_val.flags, s_val.packed, g_reply);
    g_reply_valid = 1;
    g_reply_id    = obj_id;
}

/* task_vehicle_ctrl.c run_actions(): moves counted, VC_ACT_CONFIG served */
static void run_actions(void)
{
    for (int32_t idx = 0; idx < g_actions.n; idx++)
    {
        const vc_action_t * p_a = &g_actions.a[idx];

        if ((VC_ACT_MOVE == p_a->kind) && (p_a->p1 >= 0) && (p_a->p1 < 32))
        {
            g_tally[p_a->p1]++;
        }
        else if (VC_ACT_JUNCTION == p_a->kind)
        {
            if (g_jx_n < 16)
            {
                g_jx[g_jx_n][0] = p_a->p1;
                g_jx[g_jx_n][1] = (p_a->p2 >> 8) & 0xFF;
                g_jx[g_jx_n][2] = p_a->p2 & 3;
                g_jx_n++;
            }
        }
        else if (VC_ACT_CONFIG == p_a->kind)
        {
            uint16_t          obj_id;
            vc_bcmap_result_t result;

            if (vc_bcmap_service(0 != p_a->p1, &obj_id, &result))
            {
                reply_now(S1_CFG_OP_WRITE, obj_id, (uint8_t)result);
            }
        }
    }
}

static void feed(vc_source_t src, vc_event_type_t type, int32_t p1_val,
                 int32_t p2_val)
{
    vc_event_t event = { 0 };

    event.src  = (uint8_t)src;
    event.type = (uint8_t)type;
    event.p1   = p1_val;
    event.p2   = p2_val;
    vc_sm_handle(&g_sm, &event, g_now_ms, &g_actions);
    run_actions();
}

/* exactly what vc_bridge_s3.c builds for a real S3 decode */
static void feed_s3_decode(const char * p_text)
{
    vc_event_t event = { 0 };
    size_t     idx2;

    event.src  = VC_SRC_LINE;
    event.type = VC_EV_BARCODE_DECODED;
    event.p1   = 0; /* S3's legacy nav byte: S3_BC_CHAR_* are all off */
    event.p2   = 0; /* read forwards */
    for (idx2 = 0u;
         (idx2 < sizeof(event.barcode_text) - 1u) && (p_text[idx2] != '\0');
         idx2++)
    {
        event.barcode_text[idx2] = p_text[idx2];
    }
    event.barcode_text[idx2] = '\0';
    vc_sm_handle(&g_sm, &event, g_now_ms, &g_actions);
    run_actions();
}

static void boot_controller(void)
{
    g_now_ms = 1000u;
    vc_sm_init(&g_sm, g_now_ms);
}

static void start_mission(void)
{
    boot_controller();
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
    feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
    feed(VC_SRC_OBST, VC_EV_READY, 0, 0);
}

static void junction_to_execute(void)
{
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    if (VC_EXEC_ADVANCE == g_sm.exec_phase)
    {
        feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0); /* axle advance */
    }
}

/* A U-turn in EXECUTE_ACTION from the fast pivot to the return line, as the
   bridges report it: S2's odometer climbs, S3 samples arrive fresh, the
   line passes under C during the slow approach, the wheels stop with the
   tape between C and R.  (The details: test_vc_sm.c UT-xx.) */
static void uturn_to_line(void)
{
    static uint32_t odo    = 20000u;
    vc_event_t      event2 = { 0 };
    int32_t         count;

    feed(VC_SRC_VC, VC_EV_TICK, 0, (int32_t)odo);
    odo += VC_PIVOT_COUNTS(VC_UTURN_FAST_DEG);
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, (int32_t)odo);
    event2.src  = VC_SRC_LINE;
    event2.type = VC_EV_LINE_SAMPLE;
    for (count = 0; count < 30; count++)
    {
        g_now_ms += 10u;
        odo += 8u;
        event2.p1      = ((count >= 10) && (count < 14)) ? VC_LINE_C : 0;
        event2.p2      = (int32_t)odo;
        event2.time_ms = g_now_ms;
        vc_sm_handle(&g_sm, &event2, g_now_ms, &g_actions);
        run_actions();
        if ((16 == count) && (VC_EXEC_UT_BRAKE == g_sm.exec_phase))
        {
            feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, (int32_t)odo);
        }
    }
}

static int32_t turns(void)
{
    return g_tally[VC_MOVE_TURN_LEFT] + g_tally[VC_MOVE_TURN_RIGHT] +
           g_tally[VC_MOVE_U_TURN];
}

/* the dashboard: one MQTT message through S1's decoder into the robot */
static int dashboard_write(uint16_t obj_id, uint16_t gen,
                           const uint8_t p_map[7])
{
    uint8_t      msg[S1_CFG_WRITE_LEN];
    s1_cfg_req_t req;

    (void)s1_cfg_encode_write(obj_id, gen, p_map, msg);
    if (0 != s1_cfg_decode_request(msg, (int)sizeof(msg), &req))
    {
        return 0;
    }
    g_reply_valid = 0;
    if (!vc_bcmap_post_write(req.request_id, req.expected_gen, req.map))
    {
        reply_now(S1_CFG_OP_WRITE, req.request_id, (uint8_t)VC_BCMAP_BUSY);
        return 1;
    }
    feed(VC_SRC_COMMS, VC_EV_CMD_CONFIG, (int32_t)req.request_id, 0);
    return 1;
}

/* dashboard verification: matching id, OK/UNCHANGED, all 26 entries equal */
static int reply_verifies(uint16_t obj_id, const uint8_t p_map[7])
{
    return g_reply_valid && (g_reply_id == obj_id) &&
           (S1_CFG_MARKER == g_reply[0]) &&
           ((S1_CFG_OP_REPLY | S1_CFG_OP_WRITE) == g_reply[1]) &&
           (((g_reply[2] << 8) | g_reply[3]) == obj_id) &&
           ((VC_BCMAP_OK == g_reply[4]) ||
            (VC_BCMAP_UNCHANGED == g_reply[4])) &&
           (0 == memcmp(&g_reply[8], p_map, 7));
}

/* ---- tests ---------------------------------------------------------- */

static void test_codec(void)
{
    uint8_t              p_val[7];
    navigation_command_t nav[VC_BCMAP_LETTERS];
    int                  b_ok = 1;

    CHECK(
        "BCM-01",
        (0 == vc_bcmap_wire_from_nav(NAV_STRAIGHT)) &&
            (0 == vc_bcmap_wire_from_nav(NAV_NONE)) &&
            (1 == vc_bcmap_wire_from_nav(NAV_LEFT)) &&
            (2 == vc_bcmap_wire_from_nav(NAV_RIGHT)) &&
            (3 == vc_bcmap_wire_from_nav(NAV_U_TURN)) &&
            (-1 == vc_bcmap_wire_from_nav((navigation_command_t)9)) &&
            (NAV_STRAIGHT == vc_bcmap_nav_from_wire(0)) &&
            (NAV_LEFT == vc_bcmap_nav_from_wire(1)) &&
            (NAV_RIGHT == vc_bcmap_nav_from_wire(2)) &&
            (NAV_U_TURN == vc_bcmap_nav_from_wire(3)) &&
            (NAV_NONE == vc_bcmap_nav_from_wire(4)),
        "wire codes 0 straight, 1 left, 2 right, 3 U-turn map to the firmware "
        "enum explicitly (enum values differ: LEFT=1 RIGHT=2 STRAIGHT=3 U=4)");

    vc_bcmap_factory(p_val);
    CHECK("BCM-02", 0 == memcmp(p_val, FACTORY_GOLDEN, 7),
          "factory map golden bytes 00 01 00 00 00 00 20 (H left, Z right)");

    memset(p_val, 0, 7);
    map_set(p_val, 'A', 1);
    b_ok = (0x40 == p_val[0]);
    memset(p_val, 0, 7);
    map_set(p_val, 'A', 3);
    b_ok = b_ok && (0xC0 == p_val[0]);
    memset(p_val, 0, 7);
    map_set(p_val, 'Z', 3);
    b_ok = b_ok && (0x30 == p_val[6]) && (0 == p_val[5]);
    memset(p_val, 0, 7);
    map_set(p_val, 'Y', 2);
    b_ok = b_ok && (0x80 == p_val[6]);
    CHECK("BCM-03", b_ok,
          "golden: A left = 40 ..; A U-turn = C0 ..; Z U-turn = .. 30; "
          "Y right = .. 80");

    memset(p_val, 0, 7);
    map_set(p_val, 'D', 2);
    map_set(p_val, 'E', 1); /* byte 0 / 1 boundary */
    map_set(p_val, 'H', 3);
    map_set(p_val, 'I', 2); /* byte 1 / 2 */
    map_set(p_val, 'L', 1);
    map_set(p_val, 'M', 3); /* byte 2 / 3 */
    map_set(p_val, 'P', 2);
    map_set(p_val, 'Q', 1); /* byte 3 / 4 */
    map_set(p_val, 'T', 3);
    map_set(p_val, 'U', 1); /* byte 4 / 5 */
    map_set(p_val, 'X', 1);
    map_set(p_val, 'Y', 2); /* byte 5 / 6 */
    {
        static const uint8_t golden[7] = { 0x02, 0x43, 0x81, 0xC2,
                                           0x43, 0x41, 0x80 };
        CHECK("BCM-04", 0 == memcmp(p_val, golden, 7),
              "golden across every byte boundary: D|E H|I L|M P|Q T|U X|Y = "
              "02 43 81 C2 43 41 80");
    }

    b_ok = 1;
    for (uint32_t letter = 0u; letter < VC_BCMAP_LETTERS; letter++)
    {
        for (uint32_t code = 0u; code < 4u; code++)
        {
            uint8_t back[7];
            memset(p_val, 0, 7);
            map_set(p_val, (char)('A' + (int)letter), code);
            if ((0 != vc_bcmap_unpack(p_val, nav)) ||
                (0 != vc_bcmap_pack(nav, back)) ||
                (0 != memcmp(p_val, back, 7)) ||
                (vc_bcmap_packed_code(p_val, letter) != code) ||
                (nav[letter] != vc_bcmap_nav_from_wire(code)))
            {
                b_ok = 0;
            }
            for (uint32_t other = 0u; other < VC_BCMAP_LETTERS; other++)
            {
                if ((other != letter) && (NAV_STRAIGHT != nav[other]))
                {
                    b_ok = 0;
                }
            }
        }
    }
    CHECK("BCM-05", b_ok,
          "all 26 letters x 4 actions: pack/unpack round trip, "
          "only that letter changes");

    memset(p_val, 0, 7);
    p_val[6] = 0x01;
    nav[0]   = NAV_LEFT;
    CHECK("BCM-06", (-1 == vc_bcmap_unpack(p_val, nav)) && (NAV_LEFT == nav[0]),
          "padding bits (byte 6 bits 3-0) set: rejected, output untouched");
}

static void test_record(void)
{
    uint8_t              rec[VC_BCMAP_RECORD_BYTES];
    uint8_t              back[7];
    uint32_t             gen                           = 0u;
    int                  every_byte                    = 1;
    static const uint8_t golden[VC_BCMAP_RECORD_BYTES] = {
        'B',  'C',  'M',  '1',  1,    26,   7,    0, 0x05, 0, 0, 0,
        0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x20, 0, 0,    0, 0, 0
    };
    uint8_t g_val[VC_BCMAP_RECORD_BYTES];

    CHECK("BCM-07",
          0xCBF43926u == vc_bcmap_crc32((const uint8_t *)"123456789", 9),
          "CRC-32 check value (zlib) 123456789 = CBF43926");

    vc_bcmap_record_encode(5u, FACTORY_GOLDEN, rec);
    memcpy(g_val, golden, sizeof(g_val));
    {
        uint32_t crc = vc_bcmap_crc32(g_val, 20);
        g_val[20]    = (uint8_t)crc;
        g_val[21]    = (uint8_t)(crc >> 8);
        g_val[22]    = (uint8_t)(crc >> 16);
        g_val[23]    = (uint8_t)(crc >> 24);
    }
    CHECK("BCM-08",
          (0 == memcmp(rec, g_val, sizeof(rec))) &&
              vc_bcmap_record_decode(rec, &gen, back) && (5u == gen) &&
              (0 == memcmp(back, FACTORY_GOLDEN, 7)),
          "record layout: magic BCM1, v1, 26, 7, gen LE, map, CRC-32 LE");

    for (uint32_t idx2 = 0u; idx2 < 20u; idx2++)
    {
        for (uint32_t bit = 0u; bit < 8u; bit++)
        {
            uint8_t bad[VC_BCMAP_RECORD_BYTES];
            memcpy(bad, rec, sizeof(bad));
            bad[idx2] ^= (uint8_t)(1u << bit);
            if (vc_bcmap_record_decode(bad, &gen, back))
            {
                every_byte = 0;
            }
        }
    }
    {
        uint8_t blank[VC_BCMAP_RECORD_BYTES];
        memset(blank, 0xFF, sizeof(blank));
        CHECK("BCM-09",
              every_byte && !vc_bcmap_record_decode(blank, &gen, back),
              "any single flipped bit, or an erased (FF) slot, is not a valid "
              "record");
    }
}

static void test_storage(void)
{
    uint8_t             m1_val[7];
    uint8_t             m2_val[7];
    uint8_t             m3_val[7];
    vc_bcmap_snapshot_t s_val;
    uint32_t            writes;

    /* first boot: nothing saved */
    flash_blank();
    reboot();
    s_val = snap();
    CHECK("BCM-10",
          (0u == s_val.gen) && (0 == memcmp(s_val.packed, FACTORY_GOLDEN, 7)) &&
              (0u == (s_val.flags & VC_BCMAP_FLAG_SAVED)) &&
              (0u != (s_val.flags & VC_BCMAP_FLAG_FACTORY_EQUAL)) &&
              (NAV_STRAIGHT == vc_barcode_lookup("A")) &&
              (NAV_LEFT == vc_barcode_lookup("H")) &&
              (NAV_RIGHT == vc_barcode_lookup("Z")) &&
              (NAV_STRAIGHT == vc_barcode_lookup("B")) &&
              all_letters_match(FACTORY_GOLDEN),
          "first boot, blank slots: factory map (vc_barcode_config.h), gen 0, "
          "A straight, Z right, H left, rest straight");

    /* save B = left, Z = U-turn */
    memcpy(m1_val, FACTORY_GOLDEN, 7);
    map_set(m1_val, 'B', 1);
    map_set(m1_val, 'Z', 3);
    CHECK("BCM-11",
          (VC_BCMAP_OK == vc_bcmap_replace(m1_val, 0u, false)) &&
              (1u == snap().gen) && (1u == g_flash.writes_to[0]) &&
              (0u == g_flash.writes_to[1]) && all_letters_match(m1_val) &&
              (0u != (snap().flags & VC_BCMAP_FLAG_SAVED)),
          "save: written to slot 0, read back, active, gen 1");

    reboot();
    CHECK("BCM-12",
          (1u == snap().gen) && all_letters_match(m1_val) &&
              (NAV_LEFT == vc_barcode_lookup("B")) &&
              (NAV_U_TURN == vc_barcode_lookup("Z")),
          "power cycle: the saved map is loaded again (B left, Z U-turn)");

    writes = g_flash.writes;
    CHECK("BCM-13",
          (VC_BCMAP_UNCHANGED == vc_bcmap_replace(m1_val, 1u, false)) &&
              (VC_BCMAP_UNCHANGED == vc_bcmap_replace(m1_val, 0u, false)) &&
              (writes == g_flash.writes) && (1u == snap().gen),
          "duplicate save (also with an old gen, e.g. a replayed message): "
          "UNCHANGED, flash not rewritten");

    memcpy(m2_val, m1_val, 7);
    map_set(m2_val, 'C', 2);
    CHECK("BCM-14",
          (VC_BCMAP_REJ_STALE == vc_bcmap_replace(m2_val, 0u, false)) &&
              (1u == snap().gen) && all_letters_match(m1_val) &&
              (writes == g_flash.writes),
          "write based on an older map (gen 0 while robot is at 1): STALE, "
          "nothing changes");

    CHECK("BCM-15",
          (VC_BCMAP_REJ_ACTIVE == vc_bcmap_replace(m2_val, 1u, true)) &&
              all_letters_match(m1_val) && (writes == g_flash.writes),
          "mission active: refused, map and flash untouched");

    {
        uint8_t bad[7];
        memcpy(bad, m2_val, 7);
        bad[6] |= 0x02u;
        CHECK("BCM-16",
              (VC_BCMAP_REJ_MALFORMED == vc_bcmap_replace(bad, 1u, false)) &&
                  all_letters_match(m1_val) && (writes == g_flash.writes),
              "malformed map (padding bits): refused, previous map active");
    }

    /* second save alternates to slot 1; slot 0 keeps gen 1 */
    CHECK("BCM-17",
          (VC_BCMAP_OK == vc_bcmap_replace(m2_val, 1u, false)) &&
              (1u == g_flash.writes_to[1]) && (1u == g_flash.writes_to[0]) &&
              (2u == snap().gen),
          "next save goes to the OTHER slot (A/B): the active record is never "
          "erased");

    /* interrupted writes on the third save (would go to slot 0) */
    memcpy(m3_val, m2_val, 7);
    map_set(m3_val, 'D', 3);
    {
        static const fail_t fails[]     = { FAIL_BEFORE_ERASE, FAIL_AFTER_ERASE,
                                            FAIL_HALF_PROGRAM,
                                            FAIL_SILENT_CORRUPT };
        static const char * const ids[] = { "BCM-18a", "BCM-18b", "BCM-18c",
                                            "BCM-18d" };
        static const char * const what[] = {
            "write refused before erase (other core did not park)",
            "power cut right after the erase",
            "power cut half way through programming",
            "programming reports success but read-back differs"
        };

        for (uint32_t kdx = 0u; kdx < 4u; kdx++)
        {
            char              desc[160];
            vc_bcmap_result_t r_val;
            int               live_ok;
            int               boot_ok;

            g_flash.fail_next = fails[kdx];
            r_val             = vc_bcmap_replace(m3_val, 2u, false);
            live_ok = (VC_BCMAP_REJ_STORAGE == r_val) && (2u == snap().gen) &&
                      all_letters_match(m2_val);
            reboot();
            boot_ok = (2u == snap().gen) && all_letters_match(m2_val);
            snprintf(desc, sizeof(desc),
                     "%s: STORAGE error, previous map stays "
                     "active, and is the one loaded after reboot",
                     what[kdx]);
            CHECK(ids[kdx], live_ok && boot_ok, desc);
        }
    }
    CHECK("BCM-19",
          (VC_BCMAP_OK == vc_bcmap_replace(m3_val, 2u, false)) &&
              (3u == snap().gen) && all_letters_match(m3_val),
          "after the failures, the same save succeeds normally");

    /* a corrupted slot at boot: the other one is used */
    g_flash.slot[0][14] ^= 0x10u; /* slot 0 holds gen 3 now */
    reboot();
    CHECK(
        "BCM-20", (2u == snap().gen) && all_letters_match(m2_val),
        "boot with the newest slot corrupted: the older valid slot is loaded");
    g_flash.slot[1][3] = 'X';
    reboot();
    CHECK("BCM-21",
          (0u == snap().gen) && all_letters_match(FACTORY_GOLDEN) &&
              (0u == (snap().flags & VC_BCMAP_FLAG_SAVED)),
          "both slots invalid: factory defaults");
    g_flash.read_error = 1;
    reboot();
    g_flash.read_error = 0;
    CHECK("BCM-22", (0u == snap().gen) && all_letters_match(FACTORY_GOLDEN),
          "storage unreadable at boot: factory defaults");

    /* Restore defaults on the dashboard = save the factory map */
    flash_blank();
    reboot();
    (void)vc_bcmap_replace(m1_val, 0u, false);
    CHECK("BCM-23",
          (VC_BCMAP_OK == vc_bcmap_replace(FACTORY_GOLDEN, 1u, false)) &&
              all_letters_match(FACTORY_GOLDEN) &&
              (0u != (snap().flags & VC_BCMAP_FLAG_SAVED)) &&
              (0u != (snap().flags & VC_BCMAP_FLAG_FACTORY_EQUAL)),
          "restore defaults then save: factory map stored and active");

    /* no storage on the build: RAM only, said in the flags */
    vc_bcmap_init(0);
    CHECK(
        "BCM-24",
        (0u != (snap().flags & VC_BCMAP_FLAG_NO_STORAGE)) &&
            (VC_BCMAP_OK == vc_bcmap_replace(m1_val, 0u, false)) &&
            all_letters_match(m1_val),
        "no persistent storage: map works in RAM and replies flag NO_STORAGE");
}

static void test_protocol(void)
{
    uint8_t      msg[16];
    s1_cfg_req_t req;
    int          bad = 0;

    (void)s1_cfg_encode_read(0x1234u, msg);
    CHECK("CFG-01",
          (0xC1 == msg[0]) && (0x01 == msg[1]) && (0x12 == msg[2]) &&
              (0x34 == msg[3]) && (0 == s1_cfg_decode_request(msg, 4, &req)) &&
              (S1_CFG_OP_READ == req.op) && (0x1234u == req.request_id),
          "READ request golden: C1 01 12 34");

    {
        static const uint8_t golden[13] = { 0xC1, 0x02, 0xBE, 0xEF, 0x00,
                                            0x07, 0x00, 0x01, 0x00, 0x00,
                                            0x00, 0x00, 0x20 };
        (void)s1_cfg_encode_write(0xBEEFu, 7u, FACTORY_GOLDEN, msg);
        CHECK("CFG-02",
              (0 == memcmp(msg, golden, 13)) &&
                  (0 == s1_cfg_decode_request(msg, 13, &req)) &&
                  (S1_CFG_OP_WRITE == req.op) && (7u == req.expected_gen) &&
                  (0 == memcmp(req.map, FACTORY_GOLDEN, 7)),
              "WRITE request golden: C1 02 BE EF 00 07 00 01 00 00 00 00 20");
    }
    {
        static const uint8_t golden[15] = { 0xC1, 0x82, 0xBE, 0xEF, 0x01,
                                            0x00, 0x07, 0x05, 0x00, 0x01,
                                            0x00, 0x00, 0x00, 0x00, 0x20 };
        CHECK("CFG-03",
              (15 == s1_cfg_encode_reply(S1_CFG_OP_WRITE, 0xBEEFu, 1u, 7u,
                                         0x05u, FACTORY_GOLDEN, msg)) &&
                  (0 == memcmp(msg, golden, 15)),
              "reply golden: C1 82 id result gen flags map");
    }

    (void)s1_cfg_encode_write(0x0102u, 1u, FACTORY_GOLDEN, msg);
    bad += (-1 != s1_cfg_decode_request(msg, 12, &req));
    bad += (-1 != s1_cfg_decode_request(msg, 14, &req));
    bad += (-1 != s1_cfg_decode_request(msg, 4, &req));
    bad += (-1 != s1_cfg_decode_request(msg, 3, &req));
    bad += (-1 != s1_cfg_decode_request(msg, -1, &req));
    bad += (-1 != s1_cfg_decode_request(0, 13, &req));
    msg[0] = 0xC2;
    bad += (-1 != s1_cfg_decode_request(msg, 13, &req));
    msg[0] = 0xC1;
    msg[1] = 0x03;
    bad += (-1 != s1_cfg_decode_request(msg, 13, &req));
    msg[1] = 0x02;
    msg[2] = 0;
    msg[3] = 0;
    bad += (-1 != s1_cfg_decode_request(msg, 13, &req));
    (void)s1_cfg_encode_read(0x0101u, msg);
    bad += (-1 != s1_cfg_decode_request(msg, 5, &req));
    CHECK("CFG-04", 0 == bad,
          "malformed: short/long/truncated, wrong marker, unknown op, id 0 -> "
          "rejected");
}

static void test_controller(void)
{
    static const vc_state_t idle_like[] = { VC_ST_IDLE, VC_ST_FAULT,
                                            VC_ST_MISSION_COMPLETE };
    uint8_t                 m_val[7];
    int                     b_ok = 1;

    CHECK("SMC-01", 0 == strcmp("CMD_CONFIG", vc_event_name(VC_EV_CMD_CONFIG)),
          "event name table includes CMD_CONFIG");

    for (uint32_t state2 = 0u; state2 < (uint32_t)VC_ST_COUNT; state2++)
    {
        vc_event_t event2 = { 0 };
        int        allow  = 0;

        boot_controller();
        g_sm.st.state = (vc_state_t)state2;
        for (uint32_t kdx = 0u; kdx < 3u; kdx++)
        {
            allow |= (idle_like[kdx] == (vc_state_t)state2);
        }
        event2.src  = VC_SRC_COMMS;
        event2.type = VC_EV_CMD_CONFIG;
        event2.p1   = 77;
        vc_sm_handle(&g_sm, &event2, g_now_ms, &g_actions);
        if (!((1 == g_actions.n) && (VC_ACT_CONFIG == g_actions.a[0].kind) &&
              (allow == g_actions.a[0].p1) && (77 == g_actions.a[0].p2) &&
              ((vc_state_t)state2 == g_sm.st.state)))
        {
            b_ok = 0;
        }
    }
    CHECK("SMC-02", b_ok,
          "CMD_CONFIG in every state: one VC_ACT_CONFIG, allowed only "
          "in IDLE / FAULT / MISSION_COMPLETE, state unchanged, no motion");

    start_mission();
    feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    {
        vc_event_t event2 = { 0 };
        event2.src        = VC_SRC_COMMS;
        event2.type       = VC_EV_CMD_CONFIG;
        vc_sm_handle(&g_sm, &event2, g_now_ms, &g_actions);
        CHECK("SMC-03",
              g_sm.st.paused && (1 == g_actions.n) &&
                  (VC_ACT_CONFIG == g_actions.a[0].kind) &&
                  (0 == g_actions.a[0].p1),
              "paused mission: config refused (still a mission), pause kept");
    }

    /* save attempt during a run, through the protocol */
    flash_blank();
    reboot();
    start_mission();
    memcpy(m_val, FACTORY_GOLDEN, 7);
    map_set(m_val, 'A', 1);
    tally_clear();
    (void)dashboard_write(0x2001u, 0u, m_val);
    CHECK("SMC-04",
          g_reply_valid && (0x2001u == g_reply_id) &&
              (VC_BCMAP_REJ_ACTIVE == g_reply[4]) &&
              (0 == memcmp(&g_reply[8], FACTORY_GOLDEN, 7)) &&
              (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              (0 == g_tally[VC_MOVE_STOP]) && (0u == g_flash.writes) &&
              all_letters_match(FACTORY_GOLDEN),
          "save during a run: robot answers REJ_ACTIVE with its unchanged map; "
          "mission and motion untouched, flash not written");

    /* a second write while one is pending is BUSY, the slot frees again */
    flash_blank();
    reboot();
    boot_controller();
    CHECK("SMC-05",
          vc_bcmap_post_write(1u, 0u, m_val) &&
              !vc_bcmap_post_write(2u, 0u, m_val),
          "one pending save at a time: a second one is refused (BUSY)");
    vc_bcmap_cancel_write();
    CHECK("SMC-06", vc_bcmap_post_write(3u, 0u, m_val),
          "cancel (VC event could not be posted) frees the slot");
    vc_bcmap_cancel_write();
}

static void test_end_to_end(void)
{
    uint8_t  m_val[7];
    uint16_t gen;

    /* dashboard: Read from robot */
    flash_blank();
    reboot();
    boot_controller();
    reply_now(S1_CFG_OP_READ, 0x0101u, VC_BCMAP_OK);
    gen = (uint16_t)((g_reply[5] << 8) | g_reply[6]);
    CHECK("E2E-01",
          (0 == memcmp(&g_reply[8], FACTORY_GOLDEN, 7)) && (0u == gen),
          "Read from robot: factory map, gen 0");

    /* set B = Turn left, Z = U-turn, Save to robot */
    memcpy(m_val, &g_reply[8], 7);
    map_set(m_val, 'B', VC_BCW_LEFT);
    map_set(m_val, 'Z', VC_BCW_UTURN);
    (void)dashboard_write(0x0202u, gen, m_val);
    CHECK("E2E-02",
          reply_verifies(0x0202u, m_val) && (VC_BCMAP_OK == g_reply[4]) &&
              (0x01u == ((g_reply[5] << 8) | g_reply[6])),
          "Save: reply id matches, OK, all 26 entries equal the draft, gen 1");
    CHECK("E2E-03", !reply_verifies(0x0101u, m_val),
          "an older reply id (the earlier read) cannot verify this save");

    /* close the dashboard, power-cycle */
    reboot();
    boot_controller();
    reply_now(S1_CFG_OP_READ, 0x0303u, VC_BCMAP_OK);
    CHECK("E2E-04", 0 == memcmp(&g_reply[8], m_val, 7),
          "after power cycle the robot reports the saved map");

    /* real-format B decode, then two junctions */
    start_mission();
    tally_clear();
    feed_s3_decode("B");
    CHECK("E2E-05",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              (NAV_LEFT == g_sm.pending_nav) &&
              (0 == strcmp("B", g_sm.st.last_barcode)) &&
              (NAV_LEFT == g_sm.st.last_nav) && (0 == turns()),
          "B decoded: robot keeps following, pending LEFT, last barcode B, no "
          "turn yet");
    junction_to_execute();
    CHECK("E2E-06",
          (1 == g_tally[VC_MOVE_TURN_LEFT]) && (1 == turns()) &&
              (1 == g_tally[VC_MOVE_STOP_CONFIRM]),
          "first junction: stops, then turns left exactly once");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("E2E-07",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              (NAV_NONE == g_sm.pending_nav),
          "turn done: instruction cleared, line following resumes");
    tally_clear();
    junction_to_execute();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("E2E-08",
          (0 == turns()) && (1 == g_tally[VC_MOVE_DRIVE_DIST]) &&
              (VC_ST_FOLLOW_LINE == g_sm.st.state),
          "second junction: normal no-barcode behaviour (straight through)");

    tally_clear();
    feed_s3_decode("Z");
    junction_to_execute();
    CHECK("E2E-09", (1 == g_tally[VC_MOVE_U_TURN]) && (1 == turns()),
          "Z = U-turn: the junction requests the 180 deg maneuver once");
    uturn_to_line(); /* the return line ends the U-turn */

    /* misread: an A sheet read as T is reported as T */
    tally_clear();
    feed_s3_decode("T");
    CHECK("E2E-10",
          (0 == strcmp("T", g_sm.st.last_barcode)) &&
              (NAV_STRAIGHT == g_sm.pending_nav),
          "misread T is reported as T (assigned action straight), not as A");

    /* the owner's physical test map: A = left, Z = straight */
    feed(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0);
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    reply_now(S1_CFG_OP_READ, 0x0404u, VC_BCMAP_OK);
    gen = (uint16_t)((g_reply[5] << 8) | g_reply[6]);
    memcpy(m_val, FACTORY_GOLDEN, 7);
    map_set(m_val, 'A', VC_BCW_LEFT);
    map_set(m_val, 'Z', VC_BCW_STRAIGHT);
    (void)dashboard_write(0x0505u, gen, m_val);
    CHECK("E2E-11", reply_verifies(0x0505u, m_val),
          "after Stop + Reset (IDLE) the test map A left / Z straight saves");
    reboot();
    start_mission();
    tally_clear();
    feed_s3_decode("A");
    junction_to_execute();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    {
        int a_left = (1 == g_tally[VC_MOVE_TURN_LEFT]) && (1 == turns());

        tally_clear();
        feed_s3_decode("Z");
        junction_to_execute();
        feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
        CHECK("E2E-12",
              a_left && (0 == turns()) && (1 == g_tally[VC_MOVE_DRIVE_DIST]) &&
                  (0 == strcmp("Z", g_sm.st.last_barcode)),
              "test map after reboot: A turns left, Z drives straight "
              "(still reported as Z)");
    }

    /* restore factory defaults and verify by read-back */
    feed(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0);
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    reply_now(S1_CFG_OP_READ, 0x0606u, VC_BCMAP_OK);
    gen = (uint16_t)((g_reply[5] << 8) | g_reply[6]);
    (void)dashboard_write(0x0707u, gen, FACTORY_GOLDEN);
    reboot();
    reply_now(S1_CFG_OP_READ, 0x0808u, VC_BCMAP_OK);
    CHECK("E2E-13",
          (0 == memcmp(&g_reply[8], FACTORY_GOLDEN, 7)) &&
              (0 != (g_reply[7] & VC_BCMAP_FLAG_FACTORY_EQUAL)),
          "restore factory defaults, power cycle, read back: factory map");

    /* failed read and junction without a barcode keep their behaviour */
    start_mission();
    tally_clear();
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    feed(VC_SRC_LINE, VC_EV_BARCODE_FAILED, 0, 0);
    junction_to_execute();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("E2E-14",
          (0 == turns()) && (NAV_NONE == g_sm.pending_nav) &&
              ('\0' == g_sm.st.last_barcode[0]),
          "failed read then junction: straight, no letter recorded");
}

/* Junction evidence for the dashboard (VC_ACT_JUNCTION -> robot/g10/jx) */
static void test_junction_notes(void)
{
    uint8_t m_val[7];
    uint8_t note[S1_JX_LEN];

    flash_blank();
    reboot();
    memcpy(m_val, FACTORY_GOLDEN, 7);
    map_set(m_val, 'B', VC_BCW_LEFT);
    map_set(m_val, 'Z', VC_BCW_UTURN);
    (void)vc_bcmap_replace(m_val, 0u, false);

    start_mission();
    tally_clear();
    feed_s3_decode("B");
    CHECK(
        "JX-01", (1 == g_jx_n) && jx_is(0, VC_JX_ASSIGNED, 'B', VC_BCW_LEFT),
        "B read: one ASSIGNED note, letter B, Turn left (from the active map)");
    tally_clear();
    junction_to_execute();
    CHECK("JX-02", (0 == g_jx_n) && (1 == g_tally[VC_MOVE_TURN_LEFT]),
          "junction reached and turn started: no COMPLETED yet (a lookup or a "
          "started turn is not a completed turn)");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("JX-03",
          (1 == g_jx_n) && jx_is(0, VC_JX_COMPLETED, 'B', VC_BCW_LEFT) &&
              (VC_ST_FOLLOW_LINE == g_sm.st.state),
          "turn finished, line following again: COMPLETED B Turn left");
    tally_clear();
    junction_to_execute();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("JX-04",
          (1 == g_jx_n) && jx_is(0, VC_JX_COMPLETED, 0, VC_BCW_STRAIGHT),
          "second junction: COMPLETED with no letter, straight (no-barcode "
          "behaviour)");

    tally_clear();
    feed_s3_decode("Z");
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 0, 0);
    CHECK("JX-05",
          (2 == g_jx_n) && jx_is(0, VC_JX_ASSIGNED, 'Z', VC_BCW_UTURN) &&
              jx_is(1, VC_JX_DROPPED, 'Z', VC_BCW_UTURN) && (0 == turns()) &&
              (NAV_NONE == g_sm.pending_nav),
          "obstacle at the junction: Z's U-turn is DROPPED, never executed");

    /* U-turn: COMPLETED only once the robot is aligned on the return line */
    start_mission();
    tally_clear();
    feed_s3_decode("Z");
    tally_clear();
    junction_to_execute();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0); /* fast pivot done */
    CHECK("JX-08",
          (0 == g_jx_n) && (1 == g_tally[VC_MOVE_U_TURN]) &&
              (0 == g_tally[VC_MOVE_DRIVE_DIST]) &&
              (VC_ST_EXECUTE_ACTION == g_sm.st.state),
          "Z: fast pivot done, still looking for the return line: no COMPLETED "
          "yet");
    feed(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0);
    start_mission();
    feed_s3_decode("Z");
    tally_clear();
    junction_to_execute();
    uturn_to_line();
    CHECK("JX-09",
          (1 == g_jx_n) && jx_is(0, VC_JX_COMPLETED, 'Z', VC_BCW_UTURN) &&
              (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              (0 == g_tally[VC_MOVE_DRIVE_DIST]),
          "Z: aligned on the return line: COMPLETED Z U-turn, line following "
          "again");
    feed(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0);

    start_mission();
    tally_clear();
    feed_s3_decode("HELLO");
    feed(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0);
    CHECK(
        "JX-06",
        (1 == g_jx_n) && jx_is(0, VC_JX_ASSIGNED, 0, VC_BCW_STRAIGHT) &&
            (VC_ST_FAULT == g_sm.st.state),
        "multi-letter text: ASSIGNED with no letter (straight); Stop sends no "
        "junction note (the state change is the evidence)");

    (void)s1_cfg_encode_junction(VC_JX_COMPLETED, (uint8_t)'B', VC_BCW_LEFT,
                                 0x0103u, note);
    CHECK("JX-07",
          (0xD1 == note[0]) && (2 == note[1]) && ('B' == note[2]) &&
              (1 == note[3]) && (0x01 == note[4]) && (0x03 == note[5]),
          "robot/g10/jx bytes: D1 02 42 01 01 03");
    flash_blank();
    reboot();
}

int main(void)
{
    test_codec();
    test_record();
    test_storage();
    test_protocol();
    test_controller();
    test_end_to_end();
    test_junction_notes();
    printf("[TEST] barcode map host SUMMARY %d passed, %d failed\n", g_passed,
           g_failed);
    return (0 != g_failed) ? 1 : 0;
}

/*** end of file ***/
