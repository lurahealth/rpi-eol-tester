from dataclasses import dataclass
import time
from enum import Enum
from gpiozero import OutputDevice


class DevicePowerSupply(int, Enum):
    VDUT_VSYS = 0b0
    VDUT_VBAT = 0b1


class VdutSelect(int, Enum):
    RASPBERRY_PI = 0b00
    VBAT_EXT = 0b01
    VBAT_ITEN = 0b10
    NONE = 0b11


class ItenSelect(int, Enum):
    VCHG_DCHG_ITEN_EXT = 0b00
    VCHG_ITEN = 0b01
    DCHG_ITEN_0_2K2 = 0b10
    DCHG_ITEN_1_249 = 0b11


ITEN_DEFAULT = ItenSelect.DCHG_ITEN_0_2K2


@dataclass(frozen=True)
class PowerPathConfig:
    vdut_sel: VdutSelect
    device_power: DevicePowerSupply
    iten_sel: ItenSelect
    joulescope_current_meas: bool
    iten_current_meas: bool


DUT_OFF_POWER_PATH = PowerPathConfig(
    vdut_sel=VdutSelect.NONE,
    device_power=DevicePowerSupply.VDUT_VSYS,
    iten_sel=ITEN_DEFAULT,
    joulescope_current_meas=True,
    iten_current_meas=False,
)


class PowerPath:
    def __init__(self, pin_config: dict[str, int]):
        self.vdut_en = OutputDevice(pin_config["pin_vdut_en"])
        self.vdut_sel = OutputDevice(pin_config["pin_vdut_sel"])
        self.vdut_src_sel0 = OutputDevice(pin_config["pin_vdut_src_sel0"])
        self.i_iten_meas_en = OutputDevice(pin_config["pin_i_iten_meas_en"])
        self.vdut_src_sel1 = OutputDevice(pin_config["pin_vdut_src_sel1"])
        self.idut_meas_en = OutputDevice(pin_config["pin_idut_meas_en"])
        self.iten_ch_dchg_sel0 = OutputDevice(pin_config["pin_iten_ch_dchg_sel0"])
        self.iten_ch_dchg_sel1 = OutputDevice(pin_config["pin_iten_ch_dchg_sel1"])

    def _set_vdut_sel(self, vdut_sel: VdutSelect):
        sel0 = vdut_sel.value & 1
        sel1 = (vdut_sel.value >> 1) & 1

        if sel0 == 1:
            self.vdut_src_sel0.on()
        else:
            self.vdut_src_sel0.off()

        if sel1 == 1:
            self.vdut_src_sel1.on()
        else:
            self.vdut_src_sel1.off()

    def _set_device_power_supply(self, device_power_supply: DevicePowerSupply):
        sel = device_power_supply.value & 1
        if sel == 1:
            self.vdut_sel.on()
        else:
            self.vdut_sel.off()

    def apply_config(self, config: PowerPathConfig):
        self.vdut_en.off()

        time.sleep(0.1)

        # First configure Joulescope measurement
        if config.joulescope_current_meas:
            self.idut_meas_en.on()
        else:
            self.idut_meas_en.off()

        # Then select vdut
        self._set_vdut_sel(config.vdut_sel)

        # Finally device power supply
        self._set_device_power_supply(config.device_power)

        time.sleep(0.1)

        self.vdut_en.on()

        # Wait a bit for things to stabilize
        time.sleep(0.5)
