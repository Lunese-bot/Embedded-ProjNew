/*
 * vc_bcmap_flash.h
 * Persistent storage for the barcode letter map on the Pico W's own QSPI
 * flash (firmware only; host tests use a fake store).
 *
 * Layout of the 2 MB flash (W25Q16, 4 KB erase sectors):
 *
 *   0x10000000 .. 0x101FDFFF   program image (linker ROM, 2040 KB)
 *   0x101FE000 .. 0x101FEFFF   barcode map slot 0 (one 24-byte record)
 *   0x101FF000 .. 0x101FFFFF   barcode map slot 1
 *
 * The linker script (etc/linker/pico_rp2040/tkernel_map.ld) stops the
 * image at __bcmap_flash_start, so a bigger program fails to link instead
 * of overwriting a saved map.  Flashing a new .uf2 only writes the image
 * blocks, so a saved map survives a firmware update.
 */
#ifndef VC_BCMAP_FLASH_H
#define VC_BCMAP_FLASH_H

#include "vc_barcode_map.h"

/* The flash store, or NULL when this build cannot write flash safely
   (the map then lives in RAM only and every reply says NO_STORAGE). */
const vc_bcmap_store_t * vc_bcmap_flash_store(void);

#endif /* VC_BCMAP_FLASH_H */
