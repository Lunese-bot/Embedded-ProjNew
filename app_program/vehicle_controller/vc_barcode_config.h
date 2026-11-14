/*
 * vc_barcode_config.h
 *
 * THE ONE PLACE THE TEAM EDITS to say what each barcode letter means.
 * One line per letter A to Z.  Change only the second value of a line.
 *
 *   NAV_LEFT      90 deg left at the next junction
 *   NAV_RIGHT     90 deg right at the next junction
 *   NAV_STRAIGHT  drive straight over the next junction
 *   NAV_U_TURN    turn round at the next junction
 *   NAV_NONE      meaning not defined: same as straight
 *
 * The course has NOT defined the letter meanings (owner, 2026-11-14).
 * Only Z = RIGHT is known, so every other letter stays NAV_NONE.
 * Do not invent a meaning: change a line only when the course or the
 * team says so.
 *
 * Multi-character codes are not configured here; the lookup matches the
 * whole decoded text, so only the single letters below match.
 * The table is const (flash) and is read once per decoded barcode.
 */
#ifndef VC_BARCODE_CONFIG_H
#define VC_BARCODE_CONFIG_H

#include "../robot_types.h"

#define VC_BARCODE_LETTER_COUNT 26u

#define VC_BARCODE_LETTER_TABLE(X) \
    X(A, NAV_NONE) \
    X(B, NAV_NONE) \
    X(C, NAV_NONE) \
    X(D, NAV_NONE) \
    X(E, NAV_NONE) \
    X(F, NAV_NONE) \
    X(G, NAV_NONE) \
    X(H, NAV_NONE) \
    X(I, NAV_NONE) \
    X(J, NAV_NONE) \
    X(K, NAV_NONE) \
    X(L, NAV_NONE) \
    X(M, NAV_NONE) \
    X(N, NAV_NONE) \
    X(O, NAV_NONE) \
    X(P, NAV_NONE) \
    X(Q, NAV_NONE) \
    X(R, NAV_NONE) \
    X(S, NAV_NONE) \
    X(T, NAV_NONE) \
    X(U, NAV_NONE) \
    X(V, NAV_NONE) \
    X(W, NAV_NONE) \
    X(X, NAV_NONE) \
    X(Y, NAV_NONE) \
    X(Z, NAV_RIGHT)

#endif /* VC_BARCODE_CONFIG_H */
