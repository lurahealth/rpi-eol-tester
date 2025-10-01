# Lura Teststand Test Sequence

1. Smoke/Short test
    1. Apply +2V63 to VBAT through MUXes, check VBAT voltage (within 1%) and current - should be <10uA.
    2. Apply +2V63 to VSYS, check VSYS voltage (within 1%) and current (unknown… should be <5mA)
2. Flash Program
    1. Apply +2V63 to VSYS
    2. Turn on LED, confirm LED is lit (user prompt)
    3. Turn off LED
    4. Check power (ble advertising pulses every 1 sec). Export plot from JS
3. VDD_TEMP and I2C continuity check
    1. Power on with UART Disabled, firmware asserts VDD_TEMP (can remove this later with PersistKey (HWtest == pass))
    2. Check that RPI_I2C_SCL and RPI_I2C_SDA are HIGH
        1. this confirms that VDD_TEMP and pullups are both working
4. initial UART test and Reset test
    1. Power on with UART enabled (firmware hi-z VDD_TEMP)
    2. establish UART link, check up-time < 3 seconds.
    3. Reset the DUT using the Pi (1 second)
    4. establish UART link, check up-time < 3 seconds.
5. Mag latch test (place in test order TBD)
    1. set VDUT = VBAT
    2. Check power <10uA
    3. Hover the magnet above the DUT for 1-2 seconds (user prompt)
    4. (device powers on) detect device by establishing UART comms
    5. send power off command to DUT (De-assert MAG_LATCH)
    6. confirm DUT power <10uA
6. ISFET Test with MOSFET
    1. put DUT into sensing mode (assert SENS_EN)
    2. Report ISFET_VOUT - doesn’t need to be within specific bounds, but will be between 0 and 2.63V
    3. Check Id and Vds on Joulescope (should be 100uA (5%) and 600mV (5%))
7. BLE packet test
    1. pi establishes BLE connection
    2. read Sensor data characteristic (Vd = 650mV 5%, Vs = 50mV 5%, Vout = TBD, Vbat = 2.63V, Vtemp = XXX)
    3. Check power on Vsys (ble connection pulses every 1 second) detail TBD. Export plot from JS
8. RF power
    1. put firmware into “RF test mode” (TBD). Send a tone at center of BT freq band, one low and one high.
    2. Prompt operator to measure and log RF power using [power meter](https://www.minicircuits.com/WebStore/RF-Smart-Power-Sensors.html#product_details).
9. Rx impedance
    1. Turn off the DUT
    2. Prompt operator to measure impedance across coil pins
    3. Prompt operator to fill in measurement
