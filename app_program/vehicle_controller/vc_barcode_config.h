/*
 * vc_barcode_config.h
 *
 * THE ONE PLACE THE TEAM EDITS to say what each barcode letter does.
 * One row per letter A to Z in the table below.  Change only the second
 * value of a row, save, rebuild (cd build_make && make NET=lwip MQTT=1 -j8)
 * and flash.  Example, make S turn left:
 *
 *      X(S, NAV_NONE)   becomes   X(S, NAV_LEFT)
 *
 * The four maneuvers (executed once, at the next confirmed junction after
 * the letter is read; decoding a letter never turns the robot by itself):
 *
 *   NAV_LEFT      turn 90 deg left
 *   NAV_RIGHT     turn 90 deg right
 *   NAV_U_TURN    turn round, 180 deg
 *   NAV_STRAIGHT  drive straight through the junction
 *
 * NAV_NONE is the existing fallback, not a fifth maneuver: it means
 * "no meaning assigned" and behaves exactly like NAV_STRAIGHT.  Current
 * defaults: A = NAV_NONE (goes straight), H = NAV_LEFT, Z = NAV_RIGHT,
 * every other letter NAV_NONE.  An unknown or multi-character read also
 * falls back to NAV_NONE.
 * After the maneuver the instruction is cleared, so a second junction goes
 * straight unless a new barcode is read.  Reset also clears it.
 * Any other value fails the build (check in vc_barcode_rules.c).
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
    X(H, NAV_LEFT) \
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
