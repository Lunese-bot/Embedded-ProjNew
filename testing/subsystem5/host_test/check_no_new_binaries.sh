#!/bin/sh
# Fails when git tracks a compiled program (ELF or Mach-O) that is not on
# the legacy list below.  Host-test binaries must stay out of commits; a
# new one belongs in .gitignore.  Added after S5M-05 committed two VC test
# binaries by mistake (2026-11-24).  Read-only: runs `git ls-files` only.
top=$(git rev-parse --show-toplevel 2>/dev/null) || {
    echo "check_no_new_binaries: not a git work tree, skipped"
    exit 0
}
cd "$top" || exit 1
# Tracked before this check existed; owned by other subsystems.
legacy="app_program/vehicle_controller/host_test/test_vc_sm
testing/subsystem3/host_test/test_s3_host
testing/subsystem4/host_test/s4_host_test"
bad=0
git ls-files -z | xargs -0 -n 200 sh -c '
for f do
    [ -f "$f" ] || continue
    m=$(od -An -tx1 -N4 "$f" 2>/dev/null | tr -d " \n")
    case "$m" in
        7f454c46|cffaedfe|cefaedfe|feedfacf|feedface|cafebabe) echo "$f" ;;
    esac
done' sh > /tmp/s5_tracked_bins.$$
while IFS= read -r f; do
    if ! printf '%s\n' "$legacy" | grep -qx "$f"; then
        echo "FAIL: compiled binary tracked by git: $f (git rm --cached it, add to .gitignore)"
        bad=1
    fi
done < /tmp/s5_tracked_bins.$$
rm -f /tmp/s5_tracked_bins.$$
[ "$bad" -eq 0 ] && echo "check_no_new_binaries: PASS"
exit "$bad"
