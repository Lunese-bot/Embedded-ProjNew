/*
 *      float_types.h
 *      Floating-point type names (BARR-C 5.4.b.i).  C99 does not define
 *      them.  Kept apart from app.h so that kernel-free code that is also
 *      built on a PC (host tests) can use them.
 */

#ifndef FLOAT_TYPES_H
#define FLOAT_TYPES_H

typedef float  float32_t;
typedef double float64_t;

#endif /* FLOAT_TYPES_H */

/*** end of file ***/
