"""
Simple CLI to control features of the Lura EoL test motherboard and DUT for one-off testing.
"""

from pathlib import Path
from .dut_mode_control import DutMode, DutModeControl
from .power_path import (
    DevicePowerSupply,
    PowerPathConfig,
    VdutSelect,
    ItenSelect,
    ITEN_DEFAULT,
    PowerPath,
)
from .joulescope_mux import (
    JoulescopeMeasurementConfig,
    JoulescopeMuxSelect,
    JoulescopeMux,
)
import yaml
import argparse

HARDWARE_CONFIG_FILE = Path(__file__).parent.parent / "hardware_config.yaml"

# Mode control utility needs to stick around to keep the level shifter enabled
mode_control = None


def dut_to_cli_mode(yaml_contents: dict):
    """Put the DUT in CLI mode"""
    pin_dut_reset = yaml_contents["pin_dut_reset"]
    pin_i2c_en_uart_en_l = yaml_contents["pin_i2c_en_uart_en_l"]
    uart_port = yaml_contents["uart_port"]
    uart_baudrate = yaml_contents["uart_baudrate"]

    global mode_control
    mode_control = DutModeControl(
        pin_dut_reset, pin_i2c_en_uart_en_l, uart_port, uart_baudrate
    )

    mode_control.reset_dut(DutMode.UART)
    print("To connect to the serial port, open another terminal and run:")
    print(f"screen {uart_port} {uart_baudrate}")


def main():
    parser = argparse.ArgumentParser(
        description="CLI utilities for one-off testing using the Lura EoL testing motherboard"
    )
    mux_settings_group = parser.add_argument_group("Mux Settings")
    mux_settings_group.add_argument(
        "--input_sel",
        type=str,
        default=DevicePowerSupply.VDUT_VSYS.name,
        help="Input MUX selection. Options: "
        + ", ".join([val.name for val in DevicePowerSupply]),
    )
    mux_settings_group.add_argument(
        "--vdut_sel",
        type=str,
        default=VdutSelect.RASPBERRY_PI.name,
        help="vDUT MUX selection. Options: "
        + ", ".join([val.name for val in VdutSelect]),
    )
    mux_settings_group.add_argument(
        "--iten_sel",
        type=str,
        default=ITEN_DEFAULT.name,
        help="ITEN MUX selection. Options: "
        + ", ".join([val.name for val in ItenSelect]),
    )
    measurement_group = parser.add_argument_group("Joulescope Measurement Settings")
    measurement_group.add_argument(
        "--meas",
        type=str,
        default=None,
        help="Joulescope MUX selection. If set, Joulescope measurements will be taken. Options: "
        + ", ".join([val.name for val in JoulescopeMuxSelect]),
    )
    measurement_group.add_argument(
        "--meas_dur",
        type=float,
        default=1.0,
        help="Duration over which to take each joulescope measurement",
    )
    parser.add_argument(
        "--uart",
        action="store_true",
        default=False,
        help="When set, reboot the DUT into UART mode after setting the muxes",
    )
    parser.add_argument
    args = parser.parse_args()

    # Load configuration from YAML files
    with open(HARDWARE_CONFIG_FILE, "r") as yamlfile:
        yaml_contents = yaml.safe_load(yamlfile.read())

    joulescope_mux = JoulescopeMux(yaml_contents["power_pins"])
    power_path = PowerPath(yaml_contents["power_pins"])

    should_measure = args.meas is not None

    power_path.apply_config(
        PowerPathConfig(
            vdut_sel=VdutSelect[args.vdut_sel],
            device_power=DevicePowerSupply[args.input_sel],
            iten_sel=ItenSelect[args.iten_sel],
            joulescope_current_meas=should_measure,
            iten_current_meas=False,  # ITEN current measurement is not yet supported
        )
    )

    joulescope_mux.apply_config(
        JoulescopeMeasurementConfig(
            mux_select=JoulescopeMuxSelect[args.meas]
            if should_measure
            else JoulescopeMuxSelect.DEVICE_UNDER_TEST,
            measure_current=True,
            measure_voltage=True,
        )
    )

    if args.uart:
        dut_to_cli_mode(yaml_contents)

    if should_measure:
        print("Connecting to Joulescope...")
        joulescope_mux.connect_joulescope()
        print(f"Beginning Joulescope measurements of {args.meas}")

    try:
        while True:
            if should_measure:
                measurement = joulescope_mux.measure(args.meas_dur)
                print(
                    f"{measurement.voltage_v}v\t{measurement.current_a}a\t{measurement.power_w}w"
                )
    except KeyboardInterrupt:
        print("")
        return


if __name__ == "__main__":
    main()
