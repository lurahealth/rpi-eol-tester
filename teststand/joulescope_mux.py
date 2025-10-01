from dataclasses import dataclass
import time
from enum import Enum
from gpiozero import OutputDevice
import joulescope
import numpy as np


class JoulescopeMuxSelect(int, Enum):
    DEVICE_UNDER_TEST = 0b00  # Current and Voltage
    FET_DRAIN_SOURCE = 0b01  # Current and Voltage
    ITEN = 0b10  # Current and Voltage
    ISFET_OUT = 0b11  # Voltage only, no current


@dataclass(frozen=True)
class JoulescopeMeasurementConfig:
    mux_select: JoulescopeMuxSelect
    measure_current: bool = True
    measure_voltage: bool = True


class JoulescopeMux:
    def __init__(self, pin_config: dict[str, int]):
        self.js_imeas_sel0 = OutputDevice(pin_config["pin_js_imeas_sel0"])
        self.js_imeas_sel1 = OutputDevice(pin_config["pin_js_imeas_sel1"])
        self.joulescope = None

    def _set_mux_select(self, mux_select: JoulescopeMuxSelect):
        sel0 = mux_select.value & 1
        sel1 = (mux_select.value >> 1) & 1

        if sel0 == 1:
            self.js_imeas_sel0.on()
        else:
            self.js_imeas_sel0.off()

        if sel1 == 1:
            self.js_imeas_sel1.on()
        else:
            self.js_imeas_sel1.off()

    def connect_joulescope(self):
        if self.joulescope is None:
            self.joulescope = joulescope.scan_require_one()
            self.joulescope.open()

    def disconnect_joulescope(self):
        if self.joulescope is not None:
            self.joulescope.close()
            self.joulescope = None

    def apply_config(self, config: JoulescopeMeasurementConfig):
        # Set mux selection
        self._set_mux_select(config.mux_select)

        # Wait for mux to settle
        time.sleep(0.1)

        # Connect to Joulescope if not already connected
        self.connect_joulescope()

        # Configure measurement parameters based on selection
        if config.mux_select == JoulescopeMuxSelect.ISFET_OUT:
            # ISFET mode: voltage only, no current measurement
            if config.measure_current:
                raise ValueError("Current measurement not available for ISFET_OUT mode")

    def measure(self, duration: float = 0.1) -> dict[str, float]:
        if self.joulescope is None:
            raise RuntimeError("Joulescope not connected. Call apply_config() first.")

        # Get current measurement
        data = self.joulescope.read(duration=duration, out_format="samples_get")

        current = np.average(data["signals"]["current"]["value"])
        voltage = np.average(data["signals"]["voltage"]["value"])
        power = np.average(data["signals"]["power"]["value"])

        return {
            "current_a": current,
            "voltage_v": voltage,
            "power_w": power,
        }
