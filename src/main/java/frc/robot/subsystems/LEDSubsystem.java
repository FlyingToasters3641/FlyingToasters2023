package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MathUtil;

public class LEDSubsystem extends SubsystemBase {

    // The CANdle device represented in the code
    private CANdle candle = new CANdle(22);

    // Two different timers for the code. Led Timer is for the coordinating of the
    // stripes while Sparkle Timer coordinates the sparkles.
    private final Timer ledTimer = new Timer();
    private final Timer sparkleTimer = new Timer();
    private final Timer greenTimer = new Timer();
    private final Timer checkTimer = new Timer();

    // Changes the status of the LED
    private boolean ledStatusSwitch;

    // Blinks the yellow and purple mode on and off
    private int ledBlink;

    // Used to communicate which colors the LED's should be turned on or off.
    private int ledStatus;

    // A list of int's which shows the specific positions of when a block or
    // blue/orange starts
    private int[] ledIndex = { 8, 45, 81, 117 };

    /*
     * Variables used to modify the
     */
    private String startLedColor;
    private boolean startLed;
    private int startLedCount;
    private boolean ledDisable;

    /*
     * Variables used to modify the sparkling of the blue-orange idling stripes
     */
    private int whiteness; // Brightness of the sparks
    // private String sparkColor; //Color of the sparks
    // private int sparkIndex; //Variable which is used to modify what place the
    // spark is being placed at
    // private int amountOfSparkLeds; //Amount of sparkles for each block

    public LEDSubsystem() {

        /*
         * This method defines starter variables for each private variable defined above
         * 
         * It also starts both timers to run for the duration of the code
         */

        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.GRBW; // set the strip type to RGB
        config.brightnessScalar = 1; // dim the LEDs to half brightness
        candle.configAllSettings(config);
        ledStatusSwitch = false;
        ledBlink = 0;
        startLed = false;
        ledStatus = 0;
        ledDisable = true;

        ledTimer.start();
        sparkleTimer.start();
        checkTimer.start();
    }

    /*
     * For refrence, the periodic function of every subsystem runs for every 20(?)
     * milliseconds
     * This allows something to constantly run for the rest of the program, for when
     * we want the LED to constantly display
     * We use variables to control which LED runs at the time, so they don't overlap
     */
    @Override
    public void periodic() {

        /*
         * This code checks if we are currently in a teleoperated state
         * 
         * If we aren't, it defaults to a solid blue color, otherwise it uses the switch
         * statement
         * This switch statement defaults to a blue and orange stripe pattern
         */

        if (RobotState.isDisabled() == true) {
            ledSwitch(4);
            ledDisable = true;
        } else if (RobotState.isEnabled() && ledDisable == true) {
            ledSwitch(1);
            ledDisable = false;
        }

        // This switch statement controls the color the LED's will be displaying at the
        // time

        switch (ledStatus) {
            case 0:
                // Turns the LED's off
                setColorSimplified("none", 0, 152);
                break;
            case 1:
                // Blue and orange flowing stripes code
                if (ledTimer.advanceIfElapsed(.085)) {
                    // For every iteration of .085 seconds, it increments the led color positions by
                    // 1. It does that for each position
                    setColorSimplified("blue", ledIndex[0], 36);
                    setColorSimplified("orange", ledIndex[1], 36);
                    setColorSimplified("blue", ledIndex[2], 36);
                    setColorSimplified("orange", ledIndex[3], 36);
                    ledIndex[0]++;
                    ledIndex[1]++;
                    ledIndex[2]++;
                    ledIndex[3]++;

                    // Allows for the LED to loop around the strips
                    if (startLed == true) {
                        setColorSimplified(startLedColor, 8, startLedCount);
                        startLedCount++;
                        if (startLedCount >= 36) {
                            startLedCount = 0;
                            startLed = false;
                        }
                    }
                    if (startLed == false) {
                        for (int i = 0; i < 4; i++) {
                            if (ledIndex[i] >= 116) {
                                if (i % 2 == 0) {
                                    startLedColor = "blue";
                                }
                                if (i % 2 == 1) {
                                    startLedColor = "orange";
                                }
                                startLed = true;
                            }
                        }
                    }

                    // Sets the position back to the LED start
                    for (int i = 0; i < 4; i++) {
                        if (ledIndex[i] > 151) {
                            ledIndex[i] = 8;
                        }

                    }
                }
                /*
                 * if (sparkleTimer.advanceIfElapsed(.15)) {
                 * amountOfSparkLeds = MathUtil.randomNum(3, 5);
                 * for (int i = 0; i < amountOfSparkLeds; i++) {
                 * sparkIndex = MathUtil.randomNum(8, 152);
                 * if (sparkIndex >= ledIndex[0] && sparkIndex <= ledIndex[0] + 36) {
                 * sparkColor = "blue-white";
                 * }
                 * if (sparkIndex >= ledIndex[1] && sparkIndex <= ledIndex[1] + 36) {
                 * sparkColor = "orange-white";
                 * }
                 * if (sparkIndex >= ledIndex[2] && sparkIndex <= ledIndex[2] + 36) {
                 * sparkColor = "blue-white";
                 * }
                 * if (sparkIndex >= ledIndex[3] && sparkIndex <= ledIndex[3] + 36) {
                 * sparkColor = "orange-white";
                 * }
                 * if (sparkIndex >= 8 && sparkIndex <= 8 + startLedCount) {
                 * if (startLedColor == "blue") {
                 * sparkColor = "blue-white";
                 * } else if (startLedColor == "orange") {
                 * sparkColor = "orange-white";
                 * }
                 * }
                 * 
                 * setColorSimplified(sparkColor, sparkIndex, 1);
                 * 
                 * }
                 * }
                 */

                break;
            case 2:
                // Blinks Yellow every .2 seconds
                if (ledTimer.advanceIfElapsed(.2)) {
                    ledBlink++;
                }

                if (ledBlink % 2 == 0) {
                    ledStatusSwitch = false;
                } else {
                    ledStatusSwitch = true;
                }

                if (ledStatusSwitch) {
                    setColorSimplified("yellow", 8, 152);
                }
                if (!ledStatusSwitch) {
                    setColorSimplified("none", 8, 152);
                }
                break;
            case 3:
                // Blinks Purple every .2 seconds
                if (ledTimer.advanceIfElapsed(.2)) {
                    ledBlink++;
                }

                if (ledBlink % 2 == 0) {
                    ledStatusSwitch = false;
                } else {
                    ledStatusSwitch = true;
                }

                if (ledStatusSwitch) {
                    setColorSimplified("purple", 8, 152);
                }
                if (!ledStatusSwitch) {
                    setColorSimplified("none", 8, 152);
                }
                break;
            case 4:
                // Solid blue
                setColorSimplified("blue", 8, 152);
                break;
            case 5:
                // blinks green every .2 seconds
                if (ledTimer.advanceIfElapsed(.2)) {
                    ledBlink++;

                    if (ledBlink % 2 == 0) {
                        setColorSimplified("green", 8, 152);
                    } else {
                        setColorSimplified("none", 8, 152);
                    }

                    if (greenTimer.advanceIfElapsed(2.2)) {
                        ledSwitch(1);
                    }
                }
        }
    }

    // public void setColor(int R, int G, int B, int W, int index, int count) {
    // candle.setLEDs(R, G, B, W, index, count);
    // }

    // This function just sets the colors using a string instead of using the
    // specific RGBW values
    private void setColorSimplified(String color, int index, int count) {
        if (color.equals("blue")) {
            candle.setLEDs(0, 80, 255, 0, index, count);
        } else if (color.equals("orange")) {
            candle.setLEDs(255, 25, 0, 0, index, count);
        } else if (color.equals("yellow")) {
            candle.setLEDs(225, 80, 0, 0, index, count);
        } else if (color.equals("purple")) {
            candle.setLEDs(35, 0, 60, 0, index, count);
        } else if (color.equals("green")) {
            candle.setLEDs(47, 125, 0, 0, index, count);
        } else if (color.equals("none")) {
            candle.setLEDs(0, 0, 0, 0, index, count);
        } else if (color.equals("blue-white")) {
            whiteness = MathUtil.randomNum(50, 100);
            candle.setLEDs(0, 80, 255, whiteness, index, count);
        } else if (color.equals("orange-white")) {
            whiteness = MathUtil.randomNum(50, 100);
            candle.setLEDs(255, 25, 0, whiteness, index, count);
        }
    }

    // Changes the status of the LED's for the Robotc container function to work
    public void ledSwitch(int status) {
        if (status == 1) { // red and orange mode
            ledStatus = 1;
        } else if (status == 2) { // yellow flash
            ledStatus = 2;
        } else if (status == 3) { // purple flash
            ledStatus = 3;
        } else if (status == 4) { // blue
            ledStatus = 4;
        } else if (status == 5) { // timed green flash
            ledStatus = 5;
            greenTimer.reset();
            greenTimer.start();
        }

    }

}