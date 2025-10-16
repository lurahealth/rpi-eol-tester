from contextlib import contextmanager
from enum import Enum
import time
from gpiozero import OutputDevice
from typing import Optional

import serial


class DutMode(str, Enum):
    UART = "uart"
    I2C = "i2c"


class DutModeControl:
    """Plug to control DUT boot mode (UART vs I2C)"""

    def __init__(
        self, pin_dut_reset: int, pin_i2c_en_uart_en_l: int, serial_port: str, baud: int
    ):
        super().__init__()
        self._mode_mux_pin = OutputDevice(pin_i2c_en_uart_en_l)
        # Note that DUT reset pin is reset-high
        self._dut_reset_pin = OutputDevice(pin_dut_reset, initial_value=False)

        self._serial_port = serial_port
        self._baud = baud

        self._current_mode: Optional[DutMode] = None

    @contextmanager
    def hold_dut_mode_for_reset(self, mode: DutMode):
        """Hold the DUT mode pin so that when reset the DUT will come up in the proper mode. Yields to allow the caller to perform the actual reset"""

        # Use a UART break condition to force the line low if required. This avoids having to re-configure the pin as a GPIO.
        ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=2)
        if mode == DutMode.UART:
            ser.break_condition = True

        # Allow the caller to reset the DUT
        yield

        time.sleep(0.1)

        ser.break_condition = False
        ser.close()

        self._current_mode = mode

    def reset_dut(self, mode: DutMode):
        """Reset the device under test in the given mode"""
        with self.hold_dut_mode_for_reset(mode):
            # Reset the device
            self._dut_reset_pin.on()
            time.sleep(0.05)
            self._dut_reset_pin.off()
            time.sleep(0.25)

    @property
    def current_mode(self):
        """Return the current mode"""
        return self._current_mode
