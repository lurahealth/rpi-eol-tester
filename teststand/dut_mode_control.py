from enum import Enum
import time
from gpiozero import OutputDevice
import openhtf as htf
from typing import Optional

import serial


class DutMode(str, Enum):
    UART = "uart"
    I2C = "i2c"


class DutModeControl(htf.plugs.BasePlug):
    """Plug to control DUT boot mode (UART vs I2C)"""

    def __init__(
        self, pin_dut_reset: int, pin_i2c_en_uart_en_l: int, serial_port: str, baud: int
    ):
        super().__init__()
        self._mode_mux_pin = OutputDevice(pin_i2c_en_uart_en_l)
        self._dut_reset_pin = OutputDevice(pin_dut_reset)

        self._serial_port = serial_port
        self._baud = baud

        self._current_mode: Optional[DutMode] = None

    def reset_dut(self, mode: DutMode):
        # Use a UART break condition to force the line low if required. This avoids having to re-configure the pin as a GPIO.
        ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=2)
        if mode == DutMode.UART:
            ser.break_condition = True

        # Reset the device
        self._dut_reset_pin.on()
        # TODO: reduce this timeout and remove the print
        print("Please press reset button")
        time.sleep(5.0)
        self._dut_reset_pin.off()

        ser.break_condition = False
        ser.close()

        self._current_mode = mode

    @property
    def current_mode(self):
        """Return the current mode"""
        return self._current_mode
