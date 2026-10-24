#!/usr/bin/env bash
# Cross-compiles minimal sketches with and without AutoTunePID for AVR
# (ATmega328P / Uno) and ESP32; the flash/RAM delta is the library footprint.
# sizeof(AutoTunePID) on target is recovered from an intentional template
# error message (works on any toolchain, no runtime needed).
set -euo pipefail
cd "$(dirname "$0")/.."

# Libraries dir for arduino-cli --libraries: the parent of this repository
# when the project lives inside it (research/), else the sketchbook path.
if [[ -f ../library.properties ]]; then
    DEFAULT_LIBDIR="$(cd ../.. && pwd)"
else
    DEFAULT_LIBDIR="/home/azzar/project/microcontrollers/Arduino/libraries"
fi
LIB="${ATP_LIB:-$DEFAULT_LIBDIR}"
OUT=results/mcu_footprint.txt
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

write_base() {
    mkdir -p "$TMP/base"
    printf 'void setup() {}\nvoid loop() {}\n' > "$TMP/base/base.ino"
}

write_lib() {
    mkdir -p "$TMP/lib"
    cat > "$TMP/lib/lib.ino" <<'EOF'
#include <AutoTunePID.h>
atp::AutoTunePID pid(0.0f, 255.0f, atp::TuningMethod::ZieglerNichols);
void setup() { pid.setSetpoint(100.0f); }
void loop()  { pid.update(50.0f, 0.02f); }
EOF
}

write_probe() {
    mkdir -p "$TMP/probe"
    cat > "$TMP/probe/probe.ino" <<'EOF'
#include <AutoTunePID.h>
template<int N> struct SizeProbe;
SizeProbe<sizeof(atp::AutoTunePID)> probe;
void setup() {}
void loop() {}
EOF
}

size_of_elf() { # $1 = build dir, $2 = avr-size|xtensa size tool fallback
    local elf
    elf=$(find "$1" -name '*.elf' | head -1)
    "$2" -B "$elf" | awk 'NR==2 {print $1, $2, $3}' # text data bss
}

probe_size() { # $1 = fqbn; prints sizeof from error message
    local log
    log=$(mktemp)
    if arduino-cli compile --fqbn "$1" --libraries "$LIB" "$TMP/probe" \
         --build-path "$TMP/probe/build" > "$log" 2>&1; then
        echo "PROBE_UNEXPECTEDLY_OK"
    else
        grep -oE 'SizeProbe<[0-9]+L?>' "$log" | head -1 | grep -oE '[0-9]+' || echo "PROBE_NOT_FOUND"
    fi
    rm -f "$log"
}

{
write_base
write_lib
write_probe

for target in "AVR_ATmega328P:arduino:avr:uno:avr-size" "ESP32:esp32:esp32:esp32:avr-size"; do
    name=${target%%:*}
    rest=${target#*:}
    fqbn=$(echo "$rest" | cut -d: -f1-3)
    sizetool=$(echo "$rest" | cut -d: -f4)
    echo "== $name (FQBN $fqbn) =="
    for variant in base lib; do
        arduino-cli compile --fqbn "$fqbn" --libraries "$LIB" \
            "$TMP/$variant" --build-path "$TMP/$variant/build-$name" >/dev/null 2>&1 \
            || { echo "$variant build FAILED"; continue; }
        read -r t d b < <(size_of_elf "$TMP/$variant/build-$name" "$sizetool")
        echo "$variant text=$t data=$d bss=$b"
    done
    echo "sizeof_probe=$(probe_size "$fqbn")"
    echo
done
} | tee "$OUT"
echo "[mcu_footprint] wrote $OUT"
