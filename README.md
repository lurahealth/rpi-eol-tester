# Lura Teststand

End-of-line test stand for Lura's M2 Sensor Board

## Setup

### Platform / OS

1. Flash Raspberry Pi OS 64-bit using the [Raspberry Pi Imager](https://www.raspberrypi.com/software/). It's a good idea to customize settings to configure the hostname, username, password, etc.

    - Host: `luratest1`
    - UN:   `lura`
    - PW:   `lura123`

1. Upgrade OS:

    ```shell
    sudo apt-get update
    sudo apt-get upgrade
    ```

1. Update kernel (Note: this is required for GPIO commanding to work):

    ```shell
    sudo rpi-update
    ```

1. Disable linux’s use of the primary serial port but enable the hardware:
    1. Start raspi-config: `sudo raspi-config`
    1. Select option 3 - Interface Options
    1. Select option I6 - Serial Port
    1. At the prompt Would you like a login shell to be accessible over serial?, answer 'No'
    1. At the prompt Would you like the serial port hardware to be enabled?, answer 'Yes'

1. Enable I2c on the Raspberry Pi
    1. Start raspi-config: `sudo raspi-config`
    1. Select I2c option
    1. Enable I2c kernel module

1. Exit raspi-config and reboot the Raspberry Pi for changes to take effect

### Test Application

1. Install uv: `curl -LsSf https://astral.sh/uv/install.sh | sh`
1. Install JLinkExe
    1. Download the [Jlink software for linux arm 64 (.deb)](https://www.segger.com/downloads/jlink/).
    1. Install the tools: `sudo apt install ./JLink_Linux_V868_arm64.deb`
1. Configure Tofupilot
    1. Navigate to [https://www.tofupilot.com/](https://www.tofupilot.com/) and sign up or log in
    1. Once you're in your project, click on get started and setup the tofupilot API key as an environment variable. Example:

        ```shell
        echo 'export TOFUPILOT_API_KEY=<YOUR_API_KEY>' >> ~/.bash_profile
        source ~/.bash_profile
        ```
    1. Create a procedure in the "Procedures" tab and copy the id to the `procedure_id` key in `config.yaml`
    1. Configure the `part_number` key in `config.yaml`

1. If using Raspberry Pi OS with a GUI, copy `Teststand Launcher.desktop` to your desktop.
1. To format python code run `uvx ruff format ./teststand`
1. Install Udev rules for joulescope:

    ```shell
    wget https://raw.githubusercontent.com/jetperch/joulescope_driver/main/99-joulescope.rules
    sudo cp 99-joulescope.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    rm 99-joulescope.rules
    ```
1. IMPORTANT: Reboot. Reloading the rules was not enough for me.

### Optional Tools

#### Teststand CLI

Included with the teststand is a CLI application for direct control over the power path, measurements, etc.

**Prerequisites**

1. Install screen: `apt install screen`.

**Running**

See `--help` for full list of options:

`uv run teststand_cli --help`

To put the m2 device in CLI mode, run: `uv run teststand_cli --uart`. The CLI will then provide a `screen` command to be run in a SEPARATE TERMINAL (don't close the teststand_cli process, it needs to stay running to hold the test board muxes in the correct states).

**Tips**

* To exit `screen`: `ctrl+a` then `k`
* All options for `teststand_cli` can be mixed-and-matched. For instance you can set the mux states and launch the uart terminal and start periodic Joulescope measurements all at once.


#### Joulescope UI

These instructions are UNTESTED and INCOMPLETE but a good start to getting the joulescope UI working on Raspberry pi if you need to (not required to run the teststand).

1. Install the Joulescope UI by following the instructions here: https://github.com/jetperch/pyjoulescope_ui. Take special note of the Qt dependencies section. Here are the steps I took to resolve my missing dependencies: https://forums.raspberrypi.com/viewtopic.php?t=375351 (navigating to download.qt.io/official_releases and looking for the most recent release /single/qt-everywhere-src-&lt;version&gt;.tar.gz)
    1. After configuring via cmake, run `cmake --build . --parallel && cmake --install .`

## Running a Test

- If you're on Desktop Raspberry Pi OS, double click `Teststand Launcher.desktop` and choose "Execute"
- Otherwise, from the repository root, run the teststand: `uv run teststand` to kickoff a test run
