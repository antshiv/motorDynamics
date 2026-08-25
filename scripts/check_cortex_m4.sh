#!/usr/bin/env sh
set -eu

cc="${ARM_CC:-arm-none-eabi-gcc}"
out_dir="${TMPDIR:-/tmp}/motorDynamics-cortex-m4"
mkdir -p "$out_dir"

for source in src/*.c; do
    object="$out_dir/$(basename "${source%.c}").o"
    "$cc" \
        -std=c11 \
        -mcpu=cortex-m4 \
        -mthumb \
        -mfpu=fpv4-sp-d16 \
        -mfloat-abi=hard \
        -ffreestanding \
        -fno-builtin \
        -Wall \
        -Wextra \
        -Wpedantic \
        -Wdouble-promotion \
        -Werror \
        -Iinclude \
        -c "$source" \
        -o "$object"
done

printf 'Cortex-M4 compile passed\n'
