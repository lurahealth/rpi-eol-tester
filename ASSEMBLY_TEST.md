# Assembly Test for Lura Health M2 Sensor Board

## Overview

The `assembly` test is designed to verify basic subassembly functionality before full integration testing. It focuses on testing core component communications and basic sensor functionality.

## Test Sequence

### 1. Connection Setup
- Prompts operator to connect the DUT subassembly
- Ensures proper physical connection

### 2. Power Setup  
- Configures Power Path to supply +2.63V to VSYS
- Initializes Joulescope for current measurement
- Allows startup delay for board initialization

### 3. Firmware Flash
- Flashes the latest firmware using JLink
- Verifies successful programming
- Required for subsequent communication tests

### 4. I2C TMP118 Communication Test
- **Purpose**: Verify I2C communication with TMP118 temperature sensor
- **Method**: Read temperature data via BLE (MCU handles I2C internally)  
- **Measurements**:
  - I2C communication successful (boolean)
  - TMP118 device ID (should be 0x1180)
  - Temperature reading in Celsius (15-35°C range)

### 5. ISFET ADC Connection Test
- **Purpose**: Verify ISFET sensor connection by reading ADC values
- **Method**: Compare ADC readings in air vs water
- **Measurements**:
  - ADC voltage in air (1.0-1.5V expected)
  - ADC voltage in water (1.2-1.8V expected) 
  - Connection verification (minimum 50mV difference required)
- **User Interaction**:
  - Prompts operator to ensure sensor is in air for first measurement
  - Prompts operator to place sensor in water for second measurement

## Usage

Run the assembly test using:
```bash
assembly
```

The test will use the same configuration file (`config.yaml`) as the main teststand but with assembly-specific thresholds and procedure ID.

## Key Differences from Main Test

- Focused on component-level verification rather than system-level testing
- Shorter test sequence optimized for subassembly validation
- Different output file naming (includes "assembly" suffix)
- Uses port 4445 for web GUI to avoid conflicts with main test
- Simplified power and measurement requirements

## Expected Results

A successful assembly test indicates:
- TMP118 temperature sensor is properly connected and communicating via I2C
- ISFET sensor shows appropriate response to environmental changes
- Basic firmware functionality is working
- Subassembly is ready for full integration testing