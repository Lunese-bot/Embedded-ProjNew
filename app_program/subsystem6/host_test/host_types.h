/*
 *	host_types.h
 *	The few micro T-Kernel types vc_sm.c needs, for a PC build only.
 *	Not used by the firmware.
 */

#ifndef HOST_TYPES_H
#define HOST_TYPES_H

#include <stdint.h>

typedef int8_t		B;
typedef int16_t		H;
typedef int32_t		W;
typedef uint8_t		UB;
typedef uint16_t	UH;
typedef uint32_t	UW;
typedef int		INT;
typedef unsigned int	UINT;
typedef int		BOOL;

#define TRUE	1
#define FALSE	0
#define LOCAL	static
#define EXPORT
#define IMPORT	extern

#endif /* HOST_TYPES_H */
