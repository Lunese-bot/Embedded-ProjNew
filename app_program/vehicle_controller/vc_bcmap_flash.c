/*
 * vc_bcmap_flash.c
 * Barcode map slots in the RP2040's QSPI flash (see vc_bcmap_flash.h).
 *
 * This firmware does not link the Pico SDK's hardware_flash / pico_flash,
 * so this file does what they do, with the boot ROM's flash functions:
 *
 *   1. The caller is the Vehicle Controller task, and only while no mission
 *      is running (vc_sm.c VC_EV_CMD_CONFIG), so no motion depends on it.
 *   2. Interrupts off on this core; the other core is parked in a loop in
 *      RAM with its interrupts off (kernel IPI, smp_rp2040.c
 *      knl_smp_flash_lockout_begin).  If it does not park within
 *      VC_BCMAP_PARK_US, nothing is written and the save reports
 *      REJ_STORAGE.
 *   3. From RAM: connect flash, leave XIP, erase the 4 KB sector, program
 *      one 256-byte page, flush the XIP cache, then re-run a RAM copy of
 *      boot2 to restore fast XIP.
 *   4. Release the other core, interrupts back on, and read the page back
 *      through the non-caching XIP alias.
 *
 * Typical stall: ~45 ms sector erase + <1 ms program (W25Q16 datasheet;
 * worst case erase 400 ms).  The other slot is never touched, so a power
 * cut or reset at any point leaves the previous map readable.
 */
#include "vc_bcmap_flash.h"

#include <stddef.h>
#include <stdint.h>

#define FLASH_XIP_BASE        0x10000000u
#define FLASH_XIP_NOCACHE     0x13000000u /* XIP_NOCACHE_NOALLOC alias */
#define FLASH_TOTAL_BYTES     (2048u * 1024u)
#define FLASH_SECTOR_BYTES    4096u
#define FLASH_PAGE_BYTES      256u
#define FLASH_BLOCK_BYTES     65536u /* as the Pico SDK passes */
#define FLASH_BLOCK_ERASE_CMD 0xD8u
#define BOOT2_WORDS           64u /* 256-byte second stage */

#define VC_BCMAP_FLASH_OFFSET \
    (FLASH_TOTAL_BYTES - (VC_BCMAP_SLOTS * FLASH_SECTOR_BYTES))
#define VC_BCMAP_PARK_US 20000u

/* Linker script: end of the program image = start of the map slots. */
extern char __bcmap_flash_start[];

/* Kernel (kernel/sysdepend/cpu/rp2040/smp_rp2040.c).  Call with this
   core's interrupts disabled.  begin returns 1 when the other core is
   parked (or there is no other core running), 0 on timeout. */
extern int  knl_smp_flash_lockout_begin(unsigned int timeout_us);
extern void knl_smp_flash_lockout_end(void);

typedef void (*rom_void_fn_t)(void);
typedef void (*rom_erase_fn_t)(uint32_t addr, size_t count, uint32_t block_size,
                               uint8_t block_cmd);
typedef void (*rom_prog_fn_t)(uint32_t addr, const uint8_t * p_data,
                              size_t count);
typedef void * (*rom_lookup_fn_t)(const uint16_t * p_table, uint32_t code);

typedef struct
{
    rom_void_fn_t  connect;  /* 'IF' connect_internal_flash */
    rom_void_fn_t  exit_xip; /* 'EX' flash_exit_xip */
    rom_erase_fn_t erase;    /* 'RE' flash_range_erase */
    rom_prog_fn_t  program;  /* 'RP' flash_range_program */
    rom_void_fn_t  flush;    /* 'FC' flash_flush_cache */
} flash_rom_t;

static flash_rom_t s_rom;
static uint32_t    s_boot2[BOOT2_WORDS];
static uint8_t     s_page[FLASH_PAGE_BYTES] __attribute__((aligned(4)));
static bool        s_b_ready;

#define RAMFUNC \
    __attribute__((section(".ramfunc.vc_bcmap"), noinline, long_call))

/* Runs from RAM while XIP is off: no flash code, no flash constants. */
static void RAMFUNC flash_op(uint32_t offset, const uint8_t * p_page,
                             const flash_rom_t * p_rom,
                             const uint32_t *    p_boot2)
{
    p_rom->connect();
    p_rom->exit_xip();
    p_rom->erase(offset, FLASH_SECTOR_BYTES, FLASH_BLOCK_BYTES,
                 FLASH_BLOCK_ERASE_CMD);
    p_rom->program(offset, p_page, FLASH_PAGE_BYTES);
    p_rom->flush();
    /* boot2 is position independent and returns to lr (Pico SDK does the
       same in flash_enable_xip_via_boot2) */
    ((rom_void_fn_t)((uintptr_t)p_boot2 + 1u))();
}

static uint32_t irq_save(void)
{
    uint32_t primask;

    __asm__ volatile("mrs %0, primask\n\tcpsid i" : "=r"(primask)::"memory");
    return primask;
}

static void irq_restore(uint32_t primask)
{
    __asm__ volatile("msr primask, %0" ::"r"(primask) : "memory");
}

static uint16_t rom_hword(uintptr_t address)
{
    /* hide the constant address from the compiler's bounds checks */
    __asm__ volatile("" : "+r"(address));
    return *(const volatile uint16_t *)address;
}

static void * rom_func(char c1_val, char c2_val)
{
    rom_lookup_fn_t  lookup  = (rom_lookup_fn_t)(uintptr_t)rom_hword(0x18u);
    const uint16_t * p_table = (const uint16_t *)(uintptr_t)rom_hword(0x14u);

    return lookup(p_table,
                  (uint32_t)(uint8_t)c1_val | ((uint32_t)(uint8_t)c2_val << 8));
}

static uint32_t slot_offset(uint32_t slot)
{
    return VC_BCMAP_FLASH_OFFSET + (slot * FLASH_SECTOR_BYTES);
}

static int32_t flash_read(void * p_ctx, uint32_t slot, uint8_t * p_buf,
                          uint32_t len)
{
    const volatile uint8_t * p_src;
    uint32_t                 idx;

    (void)p_ctx;
    if ((slot >= VC_BCMAP_SLOTS) || (len > FLASH_PAGE_BYTES))
    {
        return -1;
    }
    p_src = (const volatile uint8_t *)(uintptr_t)(FLASH_XIP_NOCACHE +
                                                  slot_offset(slot));
    for (idx = 0u; idx < len; idx++)
    {
        p_buf[idx] = p_src[idx];
    }
    return 0;
}

static int32_t flash_write(void * p_ctx, uint32_t slot, const uint8_t * p_buf,
                           uint32_t len)
{
    uint32_t primask;
    uint32_t idx;
    int32_t  b_ok;
    uint8_t  back[VC_BCMAP_RECORD_BYTES];

    (void)p_ctx;
    if ((!s_b_ready) || (slot >= VC_BCMAP_SLOTS) || (len > FLASH_PAGE_BYTES) ||
        (len > sizeof(back)))
    {
        return -1;
    }
    for (idx = 0u; idx < FLASH_PAGE_BYTES; idx++)
    {
        s_page[idx] = (idx < len) ? p_buf[idx] : 0xFFu;
    }

    primask = irq_save();
    b_ok    = knl_smp_flash_lockout_begin(VC_BCMAP_PARK_US);
    if (b_ok)
    {
        flash_op(slot_offset(slot), s_page, &s_rom, s_boot2);
        knl_smp_flash_lockout_end();
    }
    irq_restore(primask);
    if (!b_ok)
    {
        return -1; /* other core did not park: nothing was touched */
    }

    if (0 != flash_read(0, slot, back, len))
    {
        return -1;
    }
    for (idx = 0u; idx < len; idx++)
    {
        if (back[idx] != p_buf[idx])
        {
            return -1;
        }
    }
    return 0;
}

static const vc_bcmap_store_t s_store = { flash_read, flash_write, 0 };

const vc_bcmap_store_t * vc_bcmap_flash_store(void)
{
    uint32_t idx;

    if (!s_b_ready)
    {
        /* the linker must have reserved exactly the two slots */
        if ((uintptr_t)__bcmap_flash_start !=
            (uintptr_t)(FLASH_XIP_BASE + VC_BCMAP_FLASH_OFFSET))
        {
            return 0;
        }
        s_rom.connect  = (rom_void_fn_t)rom_func('I', 'F');
        s_rom.exit_xip = (rom_void_fn_t)rom_func('E', 'X');
        s_rom.erase    = (rom_erase_fn_t)rom_func('R', 'E');
        s_rom.program  = (rom_prog_fn_t)rom_func('R', 'P');
        s_rom.flush    = (rom_void_fn_t)rom_func('F', 'C');
        if ((0 == s_rom.connect) || (0 == s_rom.exit_xip) ||
            (0 == s_rom.erase) || (0 == s_rom.program) || (0 == s_rom.flush))
        {
            return 0;
        }
        for (idx = 0u; idx < BOOT2_WORDS; idx++)
        {
            s_boot2[idx] =
                ((const volatile uint32_t *)(uintptr_t)FLASH_XIP_BASE)[idx];
        }
        s_b_ready = true;
    }
    return &s_store;
}

/*** end of file ***/
