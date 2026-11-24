/*
 * vc_barcode_map.c
 *
 * The robot's active barcode letter map (see vc_barcode_map.h).
 * No kernel calls: runs unchanged in the host tests.
 */
#include "vc_barcode_map.h"
#include "vc_barcode_config.h"

/* Compiler + hardware barrier: the map is read from another task (and on
   the RP2040 possibly the other core) while the VC task replaces it. */
#if defined(__arm__)
#define BCMAP_BARRIER() __asm__ volatile ("dmb" ::: "memory")
#else
#define BCMAP_BARRIER() __atomic_thread_fence(__ATOMIC_SEQ_CST)
#endif

typedef char vc_bcmap_26_letters_fit_7_bytes
    [((VC_BCMAP_LETTERS * 2u + 7u) / 8u == VC_BCMAP_PACKED_BYTES) ? 1 : -1];
typedef char vc_bcmap_config_has_26_letters
    [(VC_BARCODE_LETTER_COUNT == VC_BCMAP_LETTERS) ? 1 : -1];

/* ---- codec ---------------------------------------------------------- */

int32_t vc_bcmap_wire_from_nav(navigation_command_t nav)
{
    int32_t code;

    switch (nav)
    {
        case NAV_NONE:     /* "no meaning assigned" = straight (factory table) */
        case NAV_STRAIGHT: code = (int32_t)VC_BCW_STRAIGHT; break;
        case NAV_LEFT:     code = (int32_t)VC_BCW_LEFT;     break;
        case NAV_RIGHT:    code = (int32_t)VC_BCW_RIGHT;    break;
        case NAV_U_TURN:   code = (int32_t)VC_BCW_UTURN;    break;
        default:           code = -1;                       break;
    }
    return code;
}

navigation_command_t vc_bcmap_nav_from_wire(uint32_t code)
{
    navigation_command_t nav;

    switch (code)
    {
        case VC_BCW_STRAIGHT: nav = NAV_STRAIGHT; break;
        case VC_BCW_LEFT:     nav = NAV_LEFT;     break;
        case VC_BCW_RIGHT:    nav = NAV_RIGHT;    break;
        case VC_BCW_UTURN:    nav = NAV_U_TURN;   break;
        default:              nav = NAV_NONE;     break;
    }
    return nav;
}

static uint32_t shift_of(uint32_t letter)
{
    return 6u - (2u * (letter % 4u));
}

uint32_t vc_bcmap_packed_code(const uint8_t packed[VC_BCMAP_PACKED_BYTES],
                              uint32_t letter)
{
    if (letter >= VC_BCMAP_LETTERS)
    {
        return 0u;
    }
    return ((uint32_t)packed[letter / 4u] >> shift_of(letter)) & 3u;
}

int32_t vc_bcmap_pack(const navigation_command_t nav[VC_BCMAP_LETTERS],
                      uint8_t out[VC_BCMAP_PACKED_BYTES])
{
    uint8_t  tmp[VC_BCMAP_PACKED_BYTES] = { 0 };
    uint32_t i;

    for (i = 0u; i < VC_BCMAP_LETTERS; i++)
    {
        int32_t code = vc_bcmap_wire_from_nav(nav[i]);
        if (code < 0)
        {
            return -1;
        }
        tmp[i / 4u] = (uint8_t)(tmp[i / 4u] | ((uint32_t)code << shift_of(i)));
    }
    for (i = 0u; i < VC_BCMAP_PACKED_BYTES; i++)
    {
        out[i] = tmp[i];
    }
    return 0;
}

static bool padding_ok(const uint8_t in[VC_BCMAP_PACKED_BYTES])
{
    /* letters 26 and 27 would sit in bits 3-0 of byte 6 */
    return 0u == (in[VC_BCMAP_PACKED_BYTES - 1u] & 0x0Fu);
}

int32_t vc_bcmap_unpack(const uint8_t in[VC_BCMAP_PACKED_BYTES],
                        navigation_command_t nav[VC_BCMAP_LETTERS])
{
    uint32_t i;

    if (!padding_ok(in))
    {
        return -1;
    }
    for (i = 0u; i < VC_BCMAP_LETTERS; i++)
    {
        nav[i] = vc_bcmap_nav_from_wire(vc_bcmap_packed_code(in, i));
    }
    return 0;
}

void vc_bcmap_factory(uint8_t out[VC_BCMAP_PACKED_BYTES])
{
    static const navigation_command_t factory[VC_BCMAP_LETTERS] = {
#define VC_FACTORY_LINE(letter, nav) nav,
        VC_BARCODE_LETTER_TABLE(VC_FACTORY_LINE)
#undef VC_FACTORY_LINE
    };
    /* every row is one of the four actions or NAV_NONE (checked at
       compile time in vc_barcode_rules.c), so this cannot fail */
    (void)vc_bcmap_pack(factory, out);
}

/* ---- storage record ------------------------------------------------- */

uint32_t vc_bcmap_crc32(const uint8_t * p_data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    uint32_t bit;

    for (i = 0u; i < len; i++)
    {
        crc ^= p_data[i];
        for (bit = 0u; bit < 8u; bit++)
        {
            crc = (0u != (crc & 1u)) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

static void put_le32(uint8_t * p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_le32(const uint8_t * p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

#define REC_CRC_OFFSET 20u

void vc_bcmap_record_encode(uint32_t gen, const uint8_t packed[VC_BCMAP_PACKED_BYTES],
                            uint8_t out[VC_BCMAP_RECORD_BYTES])
{
    uint32_t i;

    out[0] = 'B';
    out[1] = 'C';
    out[2] = 'M';
    out[3] = '1';
    out[4] = 1u;
    out[5] = (uint8_t)VC_BCMAP_LETTERS;
    out[6] = (uint8_t)VC_BCMAP_PACKED_BYTES;
    out[7] = 0u;
    put_le32(&out[8], gen);
    for (i = 0u; i < VC_BCMAP_PACKED_BYTES; i++)
    {
        out[12u + i] = packed[i];
    }
    out[19] = 0u;
    put_le32(&out[REC_CRC_OFFSET], vc_bcmap_crc32(out, REC_CRC_OFFSET));
}

bool vc_bcmap_record_decode(const uint8_t in[VC_BCMAP_RECORD_BYTES], uint32_t * p_gen,
                            uint8_t packed[VC_BCMAP_PACKED_BYTES])
{
    uint32_t i;

    if ((in[0] != 'B') || (in[1] != 'C') || (in[2] != 'M') || (in[3] != '1') ||
        (in[4] != 1u) || (in[5] != (uint8_t)VC_BCMAP_LETTERS) ||
        (in[6] != (uint8_t)VC_BCMAP_PACKED_BYTES) || (in[7] != 0u) ||
        (in[19] != 0u) ||
        (get_le32(&in[REC_CRC_OFFSET]) != vc_bcmap_crc32(in, REC_CRC_OFFSET)) ||
        (0u == get_le32(&in[8])) || /* gen 0 is the factory map, never stored */
        !padding_ok(&in[12]))
    {
        return false;
    }
    *p_gen = get_le32(&in[8]);
    for (i = 0u; i < VC_BCMAP_PACKED_BYTES; i++)
    {
        packed[i] = in[12u + i];
    }
    return true;
}

/* ---- active map ----------------------------------------------------- */

/* Seqlock: odd while the VC task is changing s_active. */
static volatile uint32_t         s_seq;
static vc_bcmap_snapshot_t       s_active;
static navigation_command_t      s_nav[VC_BCMAP_LETTERS]; /* VC task only */
static int32_t                   s_active_slot = -1;      /* -1 = factory */
static const vc_bcmap_store_t *  s_store;
static volatile bool             s_mission_active;
static bool                      s_b_init;

static bool same_packed(const uint8_t * p_a, const uint8_t * p_b)
{
    uint32_t i;

    for (i = 0u; i < VC_BCMAP_PACKED_BYTES; i++)
    {
        if (p_a[i] != p_b[i])
        {
            return false;
        }
    }
    return true;
}

static uint8_t base_flags(const uint8_t packed[VC_BCMAP_PACKED_BYTES], bool b_saved)
{
    uint8_t factory[VC_BCMAP_PACKED_BYTES];
    uint8_t flags = 0u;

    vc_bcmap_factory(factory);
    if (b_saved)
    {
        flags |= VC_BCMAP_FLAG_SAVED;
    }
    if (same_packed(packed, factory))
    {
        flags |= VC_BCMAP_FLAG_FACTORY_EQUAL;
    }
    if (0 == s_store)
    {
        flags |= VC_BCMAP_FLAG_NO_STORAGE;
    }
    return flags;
}

/* The only writer of s_active (VC task). */
static void set_active(uint32_t gen, const uint8_t packed[VC_BCMAP_PACKED_BYTES],
                       bool b_saved)
{
    uint32_t i;

    (void)vc_bcmap_unpack(packed, s_nav); /* validated by the caller */
    s_seq = s_seq + 1u;
    BCMAP_BARRIER();
    s_active.gen = gen;
    for (i = 0u; i < VC_BCMAP_PACKED_BYTES; i++)
    {
        s_active.packed[i] = packed[i];
    }
    s_active.flags = base_flags(packed, b_saved);
    BCMAP_BARRIER();
    s_seq = s_seq + 1u;
}

/* One stored slot: true + gen / packed when valid. */
static bool read_slot(uint32_t slot, uint32_t * p_gen,
                      uint8_t packed[VC_BCMAP_PACKED_BYTES])
{
    uint8_t rec[VC_BCMAP_RECORD_BYTES];

    if ((0 == s_store) || (0 == s_store->read) ||
        (0 != s_store->read(s_store->p_ctx, slot, rec, sizeof(rec))))
    {
        return false;
    }
    return vc_bcmap_record_decode(rec, p_gen, packed);
}

void vc_bcmap_init(const vc_bcmap_store_t * p_store)
{
    uint8_t  packed[VC_BCMAP_PACKED_BYTES];
    uint8_t  best[VC_BCMAP_PACKED_BYTES];
    uint32_t gen       = 0u;
    uint32_t best_gen  = 0u;
    int32_t  best_slot = -1;
    uint32_t slot;
    uint32_t i;

    s_store = p_store;
    for (slot = 0u; slot < VC_BCMAP_SLOTS; slot++)
    {
        if (read_slot(slot, &gen, packed) && (gen > best_gen))
        {
            best_gen  = gen;
            best_slot = (int32_t)slot;
            for (i = 0u; i < VC_BCMAP_PACKED_BYTES; i++)
            {
                best[i] = packed[i];
            }
        }
    }
    s_active_slot = best_slot;
    if (best_slot < 0)
    {
        vc_bcmap_factory(best);
        best_gen = 0u;
    }
    set_active(best_gen, best, best_slot >= 0);
    BCMAP_BARRIER();
    s_b_init = true;
}

navigation_command_t vc_bcmap_lookup_letter(char letter)
{
    if ((letter < 'A') || (letter > 'Z'))
    {
        return NAV_NONE;
    }
    if (!s_b_init)
    {
        vc_bcmap_init(0); /* used before the VC task started: factory map */
    }
    return s_nav[(uint32_t)(letter - 'A')];
}

bool vc_bcmap_get(vc_bcmap_snapshot_t * p_snap)
{
    uint32_t tries;
    uint32_t before;
    uint32_t i;

    if (!s_b_init)
    {
        return false; /* the VC task has not loaded the map yet */
    }
    for (tries = 0u; tries < 64u; tries++)
    {
        before = s_seq;
        BCMAP_BARRIER();
        if (0u != (before & 1u))
        {
            continue;
        }
        p_snap->gen = s_active.gen;
        for (i = 0u; i < VC_BCMAP_PACKED_BYTES; i++)
        {
            p_snap->packed[i] = s_active.packed[i];
        }
        p_snap->flags = s_active.flags;
        BCMAP_BARRIER();
        if (s_seq == before)
        {
            if (s_mission_active)
            {
                p_snap->flags |= VC_BCMAP_FLAG_MISSION_ACTIVE;
            }
            return true;
        }
    }
    return false;
}

void vc_bcmap_set_mission_active(bool b_active)
{
    s_mission_active = b_active;
}

vc_bcmap_result_t vc_bcmap_replace(const uint8_t packed[VC_BCMAP_PACKED_BYTES],
                                   uint16_t expected_gen, bool b_mission_active)
{
    navigation_command_t check[VC_BCMAP_LETTERS];
    uint8_t              rec[VC_BCMAP_RECORD_BYTES];
    uint8_t              back[VC_BCMAP_PACKED_BYTES];
    uint32_t             back_gen = 0u;
    uint32_t             new_gen;
    uint32_t             target;

    if (b_mission_active)
    {
        return VC_BCMAP_REJ_ACTIVE;
    }
    if (0 != vc_bcmap_unpack(packed, check))
    {
        return VC_BCMAP_REJ_MALFORMED;
    }
    if (same_packed(packed, s_active.packed))
    {
        /* duplicate save (or a replayed message): do not rewrite flash */
        return VC_BCMAP_UNCHANGED;
    }
    if ((uint16_t)s_active.gen != expected_gen)
    {
        return VC_BCMAP_REJ_STALE;
    }
    new_gen = s_active.gen + 1u;
    if (0 == s_store)
    {
        set_active(new_gen, packed, false); /* RAM only on this build */
        return VC_BCMAP_OK;
    }
    target = (0 == s_active_slot) ? 1u : 0u;
    vc_bcmap_record_encode(new_gen, packed, rec);
    if ((0 == s_store->write) ||
        (0 != s_store->write(s_store->p_ctx, target, rec, sizeof(rec))) ||
        !read_slot(target, &back_gen, back) || (back_gen != new_gen) ||
        !same_packed(back, packed))
    {
        /* the other slot still holds the active map; keep using it */
        return VC_BCMAP_REJ_STORAGE;
    }
    s_active_slot = (int32_t)target;
    set_active(new_gen, packed, true);
    return VC_BCMAP_OK;
}

/* ---- pending save handed from S1 to the VC task --------------------- */

typedef struct
{
    volatile uint32_t full; /* 0 empty (S1 may fill), 1 full (VC owns) */
    uint16_t          request_id;
    uint16_t          expected_gen;
    uint8_t           packed[VC_BCMAP_PACKED_BYTES];
} pending_t;

static pending_t s_pending;

bool vc_bcmap_post_write(uint16_t request_id, uint16_t expected_gen,
                         const uint8_t packed[VC_BCMAP_PACKED_BYTES])
{
    uint32_t i;

    if (0u != s_pending.full)
    {
        return false;
    }
    s_pending.request_id   = request_id;
    s_pending.expected_gen = expected_gen;
    for (i = 0u; i < VC_BCMAP_PACKED_BYTES; i++)
    {
        s_pending.packed[i] = packed[i];
    }
    BCMAP_BARRIER();
    s_pending.full = 1u;
    return true;
}

void vc_bcmap_cancel_write(void)
{
    BCMAP_BARRIER();
    s_pending.full = 0u;
}

bool vc_bcmap_service(bool b_allowed, uint16_t * p_request_id,
                      vc_bcmap_result_t * p_result)
{
    if (0u == s_pending.full)
    {
        return false;
    }
    BCMAP_BARRIER();
    *p_request_id = s_pending.request_id;
    *p_result     = vc_bcmap_replace(s_pending.packed, s_pending.expected_gen,
                                     !b_allowed);
    BCMAP_BARRIER();
    s_pending.full = 0u;
    return true;
}
