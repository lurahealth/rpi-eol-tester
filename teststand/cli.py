"""
Simple one-shot utility script to reboot the DUT into CLI mode.
"""

from pathlib import Path
from .dut_mode_control import DutMode, DutModeControl
import yaml

HARDWARE_CONFIG_FILE = Path(__file__).parent.parent / "hardware_config.yaml"

def main():
    # Load configuration from YAML files
    with open(HARDWARE_CONFIG_FILE, "r") as yamlfile:
        yaml_contents = yaml.safe_load(yamlfile.read())

    pin_dut_reset = yaml_contents["pin_dut_reset"]
    pin_i2c_en_uart_en_l = yaml_contents["pin_i2c_en_uart_en_l"]
    uart_port = yaml_contents["uart_port"]
    uart_baudrate = yaml_contents["uart_baudrate"]

    mode_control = DutModeControl(pin_dut_reset, pin_i2c_en_uart_en_l, uart_port, uart_baudrate)

    mode_control.reset_dut(DutMode.UART)
    print("\nThis script will keep running to hold the reset line high...")
    print("To connect to the serial port, open another terminal and run:")
    print(f"screen {uart_port} {uart_baudrate}")

    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("")
        return

if __name__ == "__main__":
    main()
