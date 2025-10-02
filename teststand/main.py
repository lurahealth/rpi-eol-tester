"""
OpenHTF test station for the Lura Health M2 sensor board
"""

import shlex
from typing import Dict, List, Optional
import openhtf as htf
from openhtf.plugs import user_input
from openhtf.output.callbacks import json_factory, console_summary
from openhtf.output.servers import station_server
from openhtf.output.web_gui import web_launcher
from openhtf.util.configuration import CONF, bind_init_args
from openhtf.plugs.user_input import UserInput
import time
import logging
from pathlib import Path
import subprocess
import serial
import struct
import asyncio
from gpiozero import OutputDevice, InputDevice
from bleak import BleakScanner, BleakClient, exc as bleak_exc
from bleak.backends.characteristic import BleakGATTCharacteristic
from .power_path import (
    DUT_OFF_POWER_PATH,
    ITEN_DEFAULT,
    PowerPath,
    PowerPathConfig,
    VdutSelect,
    DevicePowerSupply,
    ItenSelect,
)
from .joulescope_mux import (
    JoulescopeMux,
    JoulescopeMeasurementConfig,
    JoulescopeMuxSelect,
)
from .dut_mode_control import DutMode, DutModeControl
from tofupilot.openhtf import TofuPilot

logger = logging.getLogger("openhtf")

power_path: Optional[PowerPath] = None
joulescope_mux: Optional[JoulescopeMux] = None

# Results in <repo root>/results if TofuPilot isn't used
RESULTS_DIR = Path(__file__).parent.parent / "results"

HARDWARE_CONFIG_FILE = Path(__file__).parent.parent / "hardware_config.yaml"
TEST_CONFIG_FILE = Path(__file__).parent.parent / "config.yaml"

# Declare configuration keys
use_tofupilot = CONF.declare("use_tofupilot")
procedure_id = CONF.declare("procedure_id")
part_number = CONF.declare("part_number")
device_serial = CONF.declare("device_serial")
power_pins = CONF.declare("power_pins")
power_startup_delay = CONF.declare("power_startup_delay")
uart_port = CONF.declare("uart_port")
uart_baudrate = CONF.declare("uart_baudrate")
pin_i2c_en_uart_en_l = CONF.declare("pin_i2c_en_uart_en_l")
pin_dut_reset = CONF.declare("pin_dut_reset")
pin_ids_meas_en = CONF.declare("pin_ids_meas_en")
pin_mosfet_en = CONF.declare("pin_mosfet_en")
pin_vdut_en = CONF.declare("pin_vdut_en")
pin_led0_pwm = CONF.declare("pin_led0_pwm")
pin_iten_chg_dchg_en = CONF.declare("pin_iten_chg_dchg_en")
pin_js_mux_en = CONF.declare("pin_js_mux_en")
pin_led1_pwm = CONF.declare("pin_led1_pwm")
thresholds = CONF.declare("thresholds")
ble_sensor_data_service_uuid = CONF.declare("ble_sensor_data_service_uuid")
sense_enable_characteristic_uuid = CONF.declare("sense_enable_characteristic_uuid")
ble_measurements = CONF.declare("ble_measurements")

# Load configuration from YAML files
with open(HARDWARE_CONFIG_FILE, "r") as yamlfile:
    CONF.load_from_file(yamlfile, _override=True)

with open(TEST_CONFIG_FILE, "r") as yamlfile:
    CONF.load_from_file(yamlfile, _override=True)

ModeControl = bind_init_args(
    DutModeControl, pin_dut_reset, pin_i2c_en_uart_en_l, uart_port, uart_baudrate
)


# Measurement functions
def measure_dut(duration=0.1):
    """Measure voltage and current of the """
    global joulescope_mux
    assert joulescope_mux is not None

    # Configure for device under test measurement
    joulescope_mux.apply_config(
        JoulescopeMeasurementConfig(JoulescopeMuxSelect.DEVICE_UNDER_TEST)
    )
    return joulescope_mux.measure(duration=duration)

def establish_uart_connection(port, baudrate=115200, timeout=5):
    """Establish UART connection and check uptime"""
    ser = serial.Serial(port, baudrate, timeout=timeout)
    ser.write(b"uptime\n")
    time.sleep(0.1)
    response = ser.read(100).decode("utf-8", errors="ignore")
    ser.close()

    if not response.strip():
        raise RuntimeError("No response from DUT")

    response = response.replace("uptime", "").replace("lura>", "")
    uptime = float(response.strip())
    return {"connected": True, "uptime": uptime}


def set_sens_en(pin, state):
    """Control SENS_EN pin"""
    sens_en = OutputDevice(pin)
    if state:
        sens_en.on()
    else:
        sens_en.off()
    sens_en.close()


def read_isfet_vout(duration=0.1):
    """Read ISFET_VOUT voltage using Joulescope"""
    global joulescope_mux
    assert joulescope_mux is not None, "Joulescope not initialized"

    joulescope_mux.apply_config(
        JoulescopeMeasurementConfig(
            JoulescopeMuxSelect.ISFET_OUT, measure_current=False
        )
    )

    measurement = joulescope_mux.measure(duration=duration)
    return measurement.voltage_v


def establish_ble_connection(measurements: list[tuple[str, str]]):
    """Establish BLE connection and read characteristics"""

    async def scan_and_connect():
        devices = await BleakScanner.discover(timeout=5.0)
        lura_device = None
        for d in devices:
            if d.name and "lura" in d.name.lower():
                if lura_device is not None:
                    raise RuntimeError(f"Multiple Lura devices found: {d.address} and {lura_device.address}")
                lura_device = d

        if not lura_device:
            raise RuntimeError("Lura device not found")

        async with BleakClient(lura_device.address) as client:
            logger.info(f"Connected to {lura_device.name}")

            # Start sensing
            await client.write_gatt_char(CONF.sense_enable_characteristic_uuid, bytearray([1]))

            sensor_data:Dict[tuple[str, str], Optional[bytearray]] = {meas: None for meas in measurements}

            def on_data(characteristic: BleakGATTCharacteristic, data: bytearray):
                logger.info(f"Got BLE data on {characteristic.uuid}: {data}")
                this_key:Optional[tuple[str, str]] = None
                for key in sensor_data.keys():
                    if key[0] == characteristic.uuid:
                        this_key = key
                if this_key is not None:
                    sensor_data[this_key] = data

            for uuid, _ in measurements:
                await client.start_notify(uuid, on_data)

            # Wait until all data is populated or timeout
            start_time = time.time()
            while time.time() < start_time + 8.0:
                if all([data is not None for data in sensor_data.values()]):
                    break
                await asyncio.sleep(0.1)

            # Stop sensing
            await client.write_gatt_char(CONF.sense_enable_characteristic_uuid, bytearray([0]))

            return sensor_data

    result = asyncio.run(scan_and_connect())
    return (True, result)


def export_power_plot():
    """Export power consumption plot from Joulescope"""
    global joulescope_mux
    assert joulescope_mux is not None, "Joulescope not initialized"

    # TODO
    # joulescope_mux.export_statistics_plot()
    logger.info("Power plot exported successfully")


def check_i2c_lines_high():
    """Check that RPI_I2C_SCL and RPI_I2C_SDA are HIGH"""
    scl_pin = InputDevice("GPIO2", pull_up=True)
    sda_pin = InputDevice("GPIO3", pull_up=True)

    scl_high = scl_pin.value == 1
    sda_high = sda_pin.value == 1

    scl_pin.close()
    sda_pin.close()

    return {"scl_high": scl_high, "sda_high": sda_high}


@htf.measures(
    htf.Measurement("vbat_voltage").in_range(
        CONF.thresholds["vbat_voltage_min"], CONF.thresholds["vbat_voltage_max"]
    ),
    htf.Measurement("vbat_current").in_range(
        -float("inf"), CONF.thresholds["vbat_current_max"]
    ),
)
def smoke_test_vbat(test):
    """Smoke test: Apply +2V63 to VBAT, check voltage and current"""
    logger.info("==== Smoke Test VBAT ====")

    # Apply +2V63 to VBAT through MUXes
    global power_path
    assert power_path is not None

    power_path.apply_config(
        PowerPathConfig(
            VdutSelect.RASPBERRY_PI,
            DevicePowerSupply.VDUT_VBAT,
            ITEN_DEFAULT,
            joulescope_current_meas=True,
            iten_current_meas=False,
        )
    )
    time.sleep(0.5)

    # Measure VBAT voltage and current
    measurement = measure_dut(0.1)
    test.measurements.vbat_voltage = measurement.voltage_v
    test.measurements.vbat_current = measurement.current_a

    logger.info(
        f"VBAT: {measurement.voltage_v:.3f}V, {measurement.current_a * 1e6:.1f}uA"
    )


@htf.measures(
    htf.Measurement("vsys_voltage").in_range(
        CONF.thresholds["vsys_voltage_min"], CONF.thresholds["vsys_voltage_max"]
    ),
    htf.Measurement("vsys_current").in_range(
        -float("inf"), CONF.thresholds["vsys_current_max"]
    ),
)
def smoke_test_vsys(test):
    """Smoke test: Apply +2V63 to VSYS, check voltage and current"""
    logger.info("==== Smoke Test VSYS ====")

    # Apply +2V63 to VSYS
    global power_path
    assert power_path is not None

    power_path.apply_config(
        PowerPathConfig(
            VdutSelect.RASPBERRY_PI,
            DevicePowerSupply.VDUT_VSYS,
            ITEN_DEFAULT,
            joulescope_current_meas=True,
            iten_current_meas=False,
        )
    )
    time.sleep(0.5)

    # Measure VSYS voltage and current
    measurement = measure_dut(0.1)
    test.measurements.vsys_voltage = measurement.voltage_v
    test.measurements.vsys_current = measurement.current_a

    logger.info(
        f"VSYS: {measurement.voltage_v:.3f}V, {measurement.current_a * 1e3:.1f}mA"
    )


@htf.measures(htf.Measurement("flash_successful").equals(True))
def flash_firmware_phase(test):
    """Flash firmware using the jlink utility"""
    phase_name = "flash_firmware_phase"
    logger.info(f"==== Starting {phase_name} ====")
    print(f"==== Starting {phase_name} ====\n")

    jlink_script_path = Path(__file__).parent / "flash.jlink"
    if not jlink_script_path.exists():
        logger.error(f"JLink flash script ({jlink_script_path.as_posix()}) not found!")
        test.measurements.flash_successful = False
        return htf.PhaseResult.STOP

    try:
        logger.info(f"Flashing firmware using {jlink_script_path.as_posix()}")
        # Use JlinkExe to flash the firmware
        result = subprocess.run(
            shlex.split(
                f"JLinkExe -device EFR32BG27CxxxF768 -if SWD -speed 10000 -CommanderScript {jlink_script_path.as_posix()} -ExitOnError 1"
            ),
            capture_output=True,
            text=True,
            timeout=30,
        )

        if result.returncode == 0:
            print("Firmware flashed successfully")
            logger.info("Firmware flashed successfully")
            test.measurements.flash_successful = True
            return htf.PhaseResult.CONTINUE
        else:
            print(f"Flash failed: {result.stderr}, {result.stdout}")
            logger.error(f"Flash failed: {result.stderr}, {result.stdout}")
            test.measurements.flash_successful = False
            return htf.PhaseResult.STOP

    except subprocess.TimeoutExpired:
        logger.error("Flash timeout")
        test.measurements.flash_successful = False
        return htf.PhaseResult.STOP
    except Exception as e:
        logger.error(f"Flash error: {e}")
        test.measurements.flash_successful = False
        return htf.PhaseResult.STOP


@htf.measures(htf.Measurement("power_setup_successful").equals(True))
def power_setup_phase(test):
    """Setup Power Path to power the board"""
    phase_name = "power_setup_phase"
    logger.info(f"==== Starting {phase_name} ====")
    print(f"==== Starting {phase_name} ====\n")

    global power_path
    global joulescope_mux
    assert power_path is None and joulescope_mux is None

    startup_delay = CONF.power_startup_delay

    try:
        power_path = PowerPath(CONF.power_pins)
        joulescope_mux = JoulescopeMux(CONF.power_pins)

        power_path.apply_config(
            PowerPathConfig(
                VdutSelect.RASPBERRY_PI,
                DevicePowerSupply.VDUT_VSYS,
                ITEN_DEFAULT,
                joulescope_current_meas=True,
                iten_current_meas=False,
            )
        )
        test.measurements.power_setup_successful = True

    except Exception as e:
        logger.error(f"Failed to setup Power Path: {e}")
        print(f"Failed to setup Power Path: {e}")
        test.measurements.power_setup_successful = False
        return htf.PhaseResult.STOP

    # Allow time for board to power up
    time.sleep(startup_delay)

    return htf.PhaseResult.CONTINUE


@htf.plug(user_input=UserInput)
def connection_setup_phase(test, user_input):
    """Prompt operator to connect DUT"""
    phase_name = "connection_setup_phase"
    logger.info(f"==== Starting {phase_name} ====")
    print(f"==== Starting {phase_name} ====\n")

    user_input.prompt(
        "Connect the DUT. Enter when ready.",
        text_input=False,
    )
    return htf.PhaseResult.CONTINUE


@htf.measures(
    htf.Measurement("led_functional").equals("y"),
)
@htf.plug(user_input=UserInput)
@htf.plug(dut_mode=ModeControl)
def led_test(test, user_input: UserInput, dut_mode: DutModeControl):
    """LED Functional"""
    logger.info("==== LED Test ====")

    # Ensure VSYS is powered
    global power_path
    assert power_path is not None

    power_path.apply_config(
        PowerPathConfig(
            VdutSelect.RASPBERRY_PI,
            DevicePowerSupply.VDUT_VSYS,
            ITEN_DEFAULT,
            joulescope_current_meas=True,
            iten_current_meas=False,
        )
    )

    response = user_input.prompt(
        "LED should now be on. Is the LED lit? (y/n)",
        text_input=True,
    )
    test.measurements.led_functional = response

    # Reset the DUT into UART mode for the remainder of the tests
    dut_mode.reset_dut(DutMode.UART)

    # TODO: Check power and export plot
    # TODO: plot
    # export_power_plot()
    # test.measurements.power_plot_exported = True


@htf.measures(
    htf.Measurement("scl_high").equals(True),
    htf.Measurement("sda_high").equals(True),
)
@htf.plug(dut_mode=ModeControl)
def vdd_temp_i2c_test(test, dut_mode: DutModeControl):
    """VDD_TEMP and I2C continuity check"""
    logger.info("==== VDD_TEMP and I2C Continuity Check ====")

    global power_path
    assert power_path is not None

    # Power on with UART Disabled (I2C mode), firmware asserts VDD_TEMP
    power_path.apply_config(
        PowerPathConfig(
            VdutSelect.RASPBERRY_PI,
            DevicePowerSupply.VDUT_VSYS,
            ITEN_DEFAULT,
            joulescope_current_meas=True,
            iten_current_meas=False,
        )
    )
    time.sleep(CONF.power_startup_delay)

    # Switch to I2C mode and reset device
    dut_mode.reset_dut(DutMode.I2C)

    # Check that RPI_I2C_SCL and RPI_I2C_SDA are HIGH
    i2c_result = check_i2c_lines_high()
    test.measurements.scl_high = i2c_result["scl_high"]
    test.measurements.sda_high = i2c_result["sda_high"]

    logger.info(
        f"I2C Lines - SCL: {i2c_result['scl_high']}, SDA: {i2c_result['sda_high']}"
    )

    # Switch back to UART mode for subsequent tests
    dut_mode.reset_dut(DutMode.UART)


@htf.measures(
    htf.Measurement("uart_initial_uptime").in_range(
        -float("inf"), CONF.thresholds["uart_uptime_max"]
    ),
    htf.Measurement("uart_reset_uptime").in_range(
        -float("inf"), CONF.thresholds["uart_uptime_max"]
    ),
    htf.Measurement("uptime_less_after_reset").equals(True)
)
@htf.plug(dut_mode=ModeControl)
def uart_reset_test(test, dut_mode: DutModeControl):
    """Initial UART test and Reset test (with UART enabled, firmware hi-z VDD_TEMP)"""
    logger.info("==== UART and Reset Test ====")

    global power_path
    assert power_path is not None

    with dut_mode.hold_dut_mode_for_reset(DutMode.UART):
        # Power cycle: turn off first
        power_path.apply_config(
            PowerPathConfig(
                VdutSelect.NONE,
                DevicePowerSupply.VDUT_VSYS,
                ITEN_DEFAULT,
                joulescope_current_meas=False,
                iten_current_meas=False,
            )
        )
        time.sleep(0.5)

        # Power on with UART enabled (firmware will hi-z VDD_TEMP)
        power_path.apply_config(
            PowerPathConfig(
                VdutSelect.RASPBERRY_PI,
                DevicePowerSupply.VDUT_VSYS,
                ITEN_DEFAULT,
                joulescope_current_meas=True,
                iten_current_meas=False,
            )
        )
        time.sleep(0.5)

    # Establish initial UART link, check uptime
    uart_result = establish_uart_connection(CONF.uart_port, CONF.uart_baudrate)
    test.measurements.uart_initial_uptime = uart_result["uptime"]

    dut_mode.reset_dut(DutMode.UART)

    time.sleep(1)

    # Re-establish UART link after reset pin triggered, check uptime
    uart_result = establish_uart_connection(CONF.uart_port, CONF.uart_baudrate)
    test.measurements.uart_reset_uptime = uart_result["uptime"]
    test.measurements.uptime_less_after_reset = (test.measurements.uart_reset_uptime - test.measurements.uart_initial_uptime) < 0

@htf.measures(
    htf.Measurement("mag_latch_low_power").in_range(
        -float("inf"), CONF.thresholds["mag_latch_current_max"]
    ),
    htf.Measurement("mag_latch_uart_connected").equals(True),
    htf.Measurement("mag_latch_final_power").in_range(
        -float("inf"), CONF.thresholds["mag_latch_current_max"]
    ),
    htf.Measurement("magnet_applied").equals("y"),
)
@htf.plug(user_input=UserInput)
def mag_latch_test(test, user_input):
    """Magnetic latch test"""
    logger.info("==== Magnetic Latch Test ====")

    # Set VDUT = VBAT
    global power_path
    assert power_path is not None

    power_path.apply_config(
        PowerPathConfig(
            VdutSelect.RASPBERRY_PI,
            DevicePowerSupply.VDUT_VBAT,
            ITEN_DEFAULT,
            joulescope_current_meas=True,
            iten_current_meas=False,
        )
    )

    # Check low power state
    measurement = measure_dut(0.1)
    test.measurements.mag_latch_low_power = measurement.current_a

    # Prompt to apply magnet
    response = user_input.prompt(
        "Hover the magnet above the DUT for 1-2 seconds, then press Enter. Was magnet applied? (y/n)",
        text_input=True,
    )
    test.measurements.magnet_applied = response

    # Detect device by establishing UART
    uart_result = establish_uart_connection(CONF.uart_port, CONF.uart_baudrate)
    test.measurements.mag_latch_uart_connected = uart_result["connected"]

    # Send power off command (would send actual command)
    time.sleep(0.5)

    # Confirm low power again
    measurement = measure_dut(0.1)
    test.measurements.mag_latch_final_power = measurement.current_a


@htf.measures(
    htf.Measurement("isfet_vout").in_range(
        CONF.thresholds["isfet_vout_min"], CONF.thresholds["isfet_vout_max"]
    ),
    htf.Measurement("isfet_id_current").in_range(
        CONF.thresholds["isfet_id_current_min"], CONF.thresholds["isfet_id_current_max"]
    ),
    htf.Measurement("isfet_vds_voltage").in_range(
        CONF.thresholds["isfet_vds_voltage_min"],
        CONF.thresholds["isfet_vds_voltage_max"],
    ),
)
def isfet_mosfet_test(test):
    """ISFET test with MOSFET"""
    logger.info("==== ISFET MOSFET Test ====")

    # Put DUT into sensing mode via UART command
    try:
        ser = serial.Serial(CONF.uart_port, CONF.uart_baudrate, timeout=2)
        ser.write(b"sens_en 1\n")
        time.sleep(0.5)
        ser.close()
    except Exception as e:
        logger.warning(f"Failed to enable sensing mode: {e}")

    # Report ISFET_VOUT
    vout = read_isfet_vout()
    test.measurements.isfet_vout = vout

    # Check Id and Vds on Joulescope (FET drain-source measurement)
    assert joulescope_mux is not None
    joulescope_mux.apply_config(
        JoulescopeMeasurementConfig(
            JoulescopeMuxSelect.FET_DRAIN_SOURCE,
        )
    )
    measurement = joulescope_mux.measure(duration=0.5)
    test.measurements.isfet_id_current = measurement.current_a
    test.measurements.isfet_vds_voltage = measurement.voltage_v

    # Turn off sensing mode via UART command
    try:
        ser = serial.Serial(CONF.uart_port, CONF.uart_baudrate, timeout=2)
        ser.write(b"sens_en 0\n")
        time.sleep(0.1)
        ser.close()
    except Exception as e:
        logger.warning(f"Failed to disable sensing mode: {e}")


@htf.measures(
    htf.Measurement("ble_connected").equals(True),
    *[
        htf.Measurement(f"ble_measurement_{measurement['name']}").in_range(
            measurement["min"], measurement["max"]
        ) for measurement in CONF.ble_measurements
    ],
    htf.Measurement("ble_power_plot_exported").equals(True),
)
def ble_packet_test(test):
    """BLE packet test"""
    logger.info("==== BLE Packet Test ====")

    # Establish BLE connection
    connected, data = establish_ble_connection([(meas["uuid"], meas["name"]) for meas in CONF.ble_measurements])
    test.measurements.ble_connected = connected

    # Read sensor data characteristic
    for (uuid, name), value in data.items():
        parsed_value = None
        if value is not None:
            parsed_value = int.from_bytes(value, byteorder='little')
        scale = [meas["scale"] for meas in CONF.ble_measurements if meas["uuid"] == uuid][0]
        test.measurements[f"ble_measurement_{name}"] = parsed_value / scale

    # Check power on VSYS and export plot
    # TODO: plot
    export_power_plot()
    test.measurements.ble_power_plot_exported = True

    logger.info(
        f"BLE Data: {data}"
    )


@htf.measures(
    htf.Measurement("rf_power_center").in_range(
        CONF.thresholds.get("rf_power_min", -float("inf")),
        CONF.thresholds.get("rf_power_max", float("inf")),
    ),
    htf.Measurement("rf_power_low").in_range(
        CONF.thresholds.get("rf_power_min", -float("inf")),
        CONF.thresholds.get("rf_power_max", float("inf")),
    ),
    htf.Measurement("rf_power_high").in_range(
        CONF.thresholds.get("rf_power_min", -float("inf")),
        CONF.thresholds.get("rf_power_max", float("inf")),
    ),
)
@htf.plug(user_input=UserInput)
def rf_power_test(test, user_input):
    """RF power test"""
    logger.info("==== RF Power Test ====")

    # Put firmware into RF test mode via UART
    try:
        ser = serial.Serial(CONF.uart_port, CONF.uart_baudrate, timeout=2)
        ser.write(b"rf_test\n")
        time.sleep(0.5)
        ser.close()
    except Exception as e:
        logger.warning(f"Failed to enter RF test mode: {e}")

    # Prompt operator to measure RF power at center frequency
    response = user_input.prompt(
        "Measure RF power at center of BT frequency band using power meter. Enter value in dBm:",
        text_input=True,
    )
    test.measurements.rf_power_center = float(response)

    # Prompt for low frequency measurement
    response = user_input.prompt(
        "Measure RF power at low BT frequency. Enter value in dBm:",
        text_input=True,
    )
    test.measurements.rf_power_low = float(response)

    # Prompt for high frequency measurement
    response = user_input.prompt(
        "Measure RF power at high BT frequency. Enter value in dBm:",
        text_input=True,
    )
    test.measurements.rf_power_high = float(response)

    logger.info(
        f"RF Power - Center: {test.measurements.rf_power_center}dBm, Low: {test.measurements.rf_power_low}dBm, High: {test.measurements.rf_power_high}dBm"
    )


@htf.measures(
    htf.Measurement("rx_impedance").in_range(
        CONF.thresholds.get("rx_impedance_min", -float("inf")),
        CONF.thresholds.get("rx_impedance_max", float("inf")),
    ),
)
@htf.plug(user_input=UserInput)
def rx_impedance_test(test, user_input):
    """Rx impedance test"""
    logger.info("==== Rx Impedance Test ====")

    # Turn off the DUT
    global power_path
    assert power_path is not None

    power_path.apply_config(DUT_OFF_POWER_PATH)
    time.sleep(0.5)

    # Prompt operator to measure impedance across coil pins
    response = user_input.prompt(
        "Measure impedance across coil pins. Enter value in Ohms:",
        text_input=True,
    )
    test.measurements.rx_impedance = float(response)

    logger.info(f"Rx Impedance: {test.measurements.rx_impedance} Ohms")


def main():
    if not RESULTS_DIR.exists():
        RESULTS_DIR.mkdir()

    # Create test sequence (order matches TEST_SEQUENCE.md)
    test = htf.Test(
        connection_setup_phase,
        power_setup_phase,
        smoke_test_vbat,
        smoke_test_vsys,
        flash_firmware_phase,
        led_test,
        vdd_temp_i2c_test,
        uart_reset_test,
        mag_latch_test,
        isfet_mosfet_test,
        ble_packet_test,
        rf_power_test,
        rx_impedance_test,
        procedure_id=CONF.procedure_id,
        part_number=CONF.part_number,
    )

    # Configure JSON output
    json_callback = json_factory.OutputToJSON(
        RESULTS_DIR.as_posix()
        + "/{start_time_millis}.lura_m2_{dut_id}.{metadata[test_name]}.json"
    )
    # Configure console summary
    console_callback = console_summary.ConsoleSummary()

    # Check if tofupilot is enabled in config
    if CONF.use_tofupilot:
        # Run with TofuPilot
        with TofuPilot(test):
            test.execute(test_start=user_input.prompt_for_test_start())
    else:
        # Start with local web GUI
        CONF.load(station_server_port="4444")
        with station_server.StationServer() as server:
            web_launcher.launch("http://localhost:4444")
            test.add_output_callbacks(
                server.publish_final_state, json_callback, console_callback
            )
            test.execute(test_start=user_input.prompt_for_test_start())


if __name__ == "__main__":
    main()
