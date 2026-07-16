#!/bin/sh
set -eu

# F450 pinmux profile for Milk-V Duo 256M (SG2002)
# - IMU: I2C0 on GP0/GP1 (physical pin 1/2)
# - ESC PWM: 4 channels on GP2/GP3/GP10/GP11
# - FS-iA6B IBUS UART RX: selectable profile (default uart3 to avoid debug-console conflict)
# - ESP8266 AT link: UART2 on GP6/GP7
#
# Usage:
#   sh duo256m_pinmux_f450.sh [uart0|uart1|uart3]
# Example:
#   sh duo256m_pinmux_f450.sh uart3

UART_PROFILE="${1:-uart3}"

if ! command -v duo-pinmux >/dev/null 2>&1; then
  echo "ERROR: duo-pinmux not found" >&2
  exit 1
fi

set_pin_func() {
  pin="$1"
  shift
  funcs="$*"
  current="$(duo-pinmux -r "$pin" 2>/dev/null || true)"

  for func in $funcs; do
    echo "$current" | grep -Fq "$func" || continue
    echo "set $pin -> $func"
    duo-pinmux -w "$pin/$func"
    return 0
  done

  echo "ERROR: none of [$funcs] available on $pin" >&2
  duo-pinmux -r "$pin" || true
  exit 2
}

echo "[1/5] Configure I2C0 for MPU6050/HMC5883L"
# Different Duo firmware versions expose I2C0 as CV_* or IIC*/I2C* names.
set_pin_func GP0 CV_SCL0 IIC0_SCL I2C0_SCL
set_pin_func GP1 CV_SDA0 IIC0_SDA I2C0_SDA

echo "[2/5] Configure 4x PWM for ESC"
# Some releases expose PWM names with underscores (PWM_7), others without (PWM7).
set_pin_func GP2 PWM_7 PWM7
set_pin_func GP3 PWM_6 PWM6
set_pin_func GP10 PWM_10 PWM10
set_pin_func GP11 PWM_11 PWM11

echo "[3/5] Configure IBUS UART profile: ${UART_PROFILE}"
case "$UART_PROFILE" in
  uart0)
    # Note: GP12/GP13 are also default debug UART pins.
    set_pin_func GP12 UART0_TX UART_0_TX UART1_TX UART_1_TX
    set_pin_func GP13 UART0_RX UART_0_RX UART1_RX UART_1_RX
    ;;
  uart1)
    set_pin_func GP12 UART1_TX UART_1_TX UART0_TX UART_0_TX
    set_pin_func GP13 UART1_RX UART_1_RX UART0_RX UART_0_RX
    ;;
  uart3)
    # Uses GP4/GP5, avoids GP12/GP13 debug console pins.
    set_pin_func GP4 UART3_TX UART_3_TX UART2_TX UART_2_TX
    set_pin_func GP5 UART3_RX UART_3_RX UART2_RX UART_2_RX
    ;;
  *)
    echo "ERROR: UART profile must be one of: uart0 | uart1 | uart3" >&2
    exit 3
    ;;
esac

echo "[4/5] Configure ESP8266 UART2"
set_pin_func GP6 UART2_TX UART_2_TX
set_pin_func GP7 UART2_RX UART_2_RX

echo "[5/5] Verify effective mux"
for pin in GP0 GP1 GP2 GP3 GP10 GP11 GP4 GP5 GP6 GP7 GP12 GP13; do
  duo-pinmux -r "$pin" >/dev/null 2>&1 && duo-pinmux -r "$pin" || true
done

echo "Done."
