from dataclasses import dataclass
import time
from enum import Enum
from typing import Literal
from gpiozero import OutputDevice
import joulescope
import numpy as np


class JoulescopeMuxSelect(int, Enum):
    DEVICE_UNDER_TEST = 0b00  # Current and Voltage
    FET_DRAIN_SOURCE = 0b01  # Current and Voltage
    ITEN = 0b10  # Current and Voltage
    ISFET_OUT = 0b11  # Voltage only, no current


@dataclass(frozen=True)
class JoulescopeMeasurement:
    voltage_v: float
    current_a: float
    power_w: float


class JoulescopeMux:
    def __init__(self, pin_config: dict[str, int]):
        self.js_mux_en = OutputDevice(pin_config["pin_js_mux_en"], initial_value=False)
        self.js_imeas_sel0 = OutputDevice(pin_config["pin_js_imeas_sel0"])
        self.js_imeas_sel1 = OutputDevice(pin_config["pin_js_imeas_sel1"])
        self.i_iten_meas_en = OutputDevice(pin_config["pin_i_iten_meas_en"])
        self.ids_meas_en = OutputDevice(pin_config["pin_ids_meas_en"])
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

    def apply_config(self, mux_select: JoulescopeMuxSelect):
        # Set mux selection
        self.js_mux_en.off()

        self._set_mux_select(mux_select)

        if mux_select == JoulescopeMuxSelect.ITEN:
            self.i_iten_meas_en.on()
        else:
            self.i_iten_meas_en.off()

        if mux_select == JoulescopeMuxSelect.FET_DRAIN_SOURCE:
            self.ids_meas_en.on()
        else:
            self.ids_meas_en.off()

        time.sleep(0.1)
        self.js_mux_en.on()

        # Wait for mux to settle
        time.sleep(0.1)

        # Connect to Joulescope if not already connected
        self.connect_joulescope()


    def measure(
        self, duration: float = 0.1, output: Literal["avg", "max", "min"] = "avg"
    ) -> JoulescopeMeasurement:
        if self.joulescope is None:
            raise RuntimeError("Joulescope not connected. Call apply_config() first.")

        # Get current measurement
        data = self.joulescope.read(duration=duration, out_format="samples_get")

        if output == "avg":
            current = np.average(data["signals"]["current"]["value"])
            voltage = np.average(data["signals"]["voltage"]["value"])
            power = np.average(data["signals"]["power"]["value"])
        elif output == "max":
            current = np.max(data["signals"]["current"]["value"])
            voltage = np.max(data["signals"]["voltage"]["value"])
            power = np.max(data["signals"]["power"]["value"])
        elif output == "min":
            current = np.min(data["signals"]["current"]["value"])
            voltage = np.min(data["signals"]["voltage"]["value"])
            power = np.min(data["signals"]["power"]["value"])
        else:
            raise ValueError(f"{output} is not a valid value for output")

        return JoulescopeMeasurement(
            current_a=current,
            voltage_v=voltage,
            power_w=power,
        )
