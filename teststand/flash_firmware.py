"""Common utility for flashing firmware"""

from pathlib import Path
import logging
import shlex
import subprocess
import time

logger = logging.Logger(__name__)


def flash_firmware() -> bool:
    jlink_script_path = Path(__file__).parent / "flash.jlink"
    if not jlink_script_path.exists():
        logger.error(f"JLink flash script ({jlink_script_path.as_posix()}) not found!")
        return False

    try:
        logger.info(f"Flashing firmware using {jlink_script_path.as_posix()}")

        time.sleep(1.0)

        # Use JlinkExe to flash the firmware
        result = subprocess.run(
            shlex.split(
                f"JLinkExe -device EFR32BG27CxxxF768 -if SWD -speed 10000 -CommanderScript {jlink_script_path.as_posix()} -ExitOnError 1"
            ),
            capture_output=True,
            text=True,
            timeout=120,
        )

        if result.returncode == 0:
            print("Firmware flashed successfully")
            logger.info("Firmware flashed successfully")
            return True
        else:
            print(f"Flash failed: \n{result.stderr}\n {result.stdout}")
            logger.error(f"Flash failed: {result.stderr}, {result.stdout}")
            return False

    except subprocess.TimeoutExpired:
        logger.error("Flash timeout")
        return False
    except Exception as e:
        logger.error(f"Flash error: {e}")
        return False
