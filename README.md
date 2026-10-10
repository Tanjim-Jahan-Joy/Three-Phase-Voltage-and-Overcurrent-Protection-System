## How to Run the Project

### 1. Clone or Download the Repository

Clone the repository using Git:

``` bash
git clone <YOUR_GITHUB_REPOSITORY_URL>
```

Alternatively, download the repository as a ZIP file from GitHub and
extract it.

### 2. Open the Arduino Firmware

1.  Install the Arduino IDE.
2.  Open `voltage_current_protector.ino`.
3.  Select **Arduino Mega or Mega 2560** under Tools → Board.
4.  Check that the required libraries are installed.
5.  Click **Verify** to compile the code.

If you are running the project entirely in Proteus, you will need the
compiled firmware HEX file rather than a physical Arduino board.

### 3. Generate the HEX File

In the Arduino IDE, enable compilation output and compile the sketch.
Locate the generated `.hex` file in the build output or temporary build
directory.

In Proteus:

1.  Open the project schematic.
2.  Double-click the Arduino Mega 2560 component.
3.  Set the **Program File** field to the compiled HEX file.
4.  Confirm that the clock frequency matches the firmware configuration.
5.  Apply the changes.

### 4. Run the Proteus Simulation

1.  Open the Proteus project file (`.pdsprj` or the relevant schematic
    file).
2.  Confirm the sensor, LCD, keypad, relay, motor, and COMPIM
    connections.
3.  Start the simulation.
4.  Observe the voltage, current, and phase status on the LCD.
5.  Use the keypad to navigate the settings and change protection
    thresholds.

### 5. Test Protection Conditions

Use the simulation components to test the following conditions:

  Test condition               Expected behaviour
  ---------------------------- ---------------------------------------
  Normal voltage and current   Phase status shows normal
  Overvoltage                  Corresponding phase fault is detected
  Undervoltage                 Corresponding phase fault is detected
  Overcurrent                  Corresponding phase fault is detected
  Sensor error                 Sensor fault is indicated
  Fault in one phase           Corresponding phase relay should trip

Actual behaviour should be verified in the running simulation.

### 6. Monitor Serial Data on a PC

The Arduino Mega communicates with the PC through the Proteus COMPIM
component using its hardware UART0.

1.  Configure COMPIM to use the appropriate virtual COM port.
2.  Open PuTTY or another serial terminal.
3.  Select the corresponding COM port.
4.  Configure the baud rate to match the firmware. For example, use
    `9600` baud if the code initializes serial communication at `9600`.
5.  Set data bits to `8`, parity to `None`, stop bits to `1`, and flow
    control to `None`.
6.  Start the simulation and check whether voltage, current, phase
    status, and fault information appear in the terminal.

If the COM port is already occupied, configure a virtual COM-port pair
and use the corresponding ports for Proteus and the terminal
application.

## Configuration and Settings

The keypad and LCD provide an interface for changing the protection
thresholds for the R, Y, and B phases. The configured values are stored
in EEPROM so they can be retained after restart.

Refer to the firmware for the exact key functions, threshold limits, and
default values.

## Troubleshooting

-   **Compilation errors:** Check the selected board and ensure all
    required libraries are installed.
-   **LCD shows no text:** Verify the I2C address, SDA/SCL wiring, and
    LCD contrast.
-   **Incorrect sensor readings:** Check sensor scaling, calibration
    constants, and ADC input ranges.
-   **Relay does not trip:** Verify relay polarity, pin assignments, and
    fault detection logic.
-   **No serial output:** Check COMPIM TX/RX connections, COM-port
    configuration, and baud rate.
-   **Serial framing errors:** Confirm that the UART settings match on
    both sides and that the serial signal wiring is correct.

## Safety Notice

This project is intended for educational purposes and Proteus
simulation. Real three-phase AC systems involve hazardous voltages and
currents. Any physical implementation requires suitable isolation,
protection, properly rated components, and qualified supervision.
