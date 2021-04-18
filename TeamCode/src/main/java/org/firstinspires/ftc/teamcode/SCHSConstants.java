package org.firstinspires.ftc.teamcode;

class SCHSConstants {

    // power constants between 1 and -1
    static final double POWER_FULL = 1;
    static final double POWER_HALF = 0.5;
    static final double POWER_TURN_SPEED = 0.5;
    static final double SHOOT_VEL = 1000; //2000 = too much, fix/modify this value after testing, velocity for shooting from back wall starting position
    static final double SHOOT_VEL_NEW = 950;
    static final int WOBBLE_ARM_DIST = 310;

    //Robot Constants
    static final double ROBOT_WIDTH = 14; //inches
    static final double ROBOT_MAX_SPEED = 4; //feet/sec
    static final int DRIVE = 0;
    static final int WOBBLE = 1;
    static final int LONG_DRIVE = 3;

    //Servo Constants
    static final boolean SERVO_OPEN = true;
    static final boolean SERVO_CLOSE = false;
    static final double INCREMENT = 0.01;
    static final int CYCLE_MS = 50;

    // derived robot parameters
    // The CoreHex motor has 4 ticks per revolution and is geared down by 72
    //   those attributes should be in the DcMotor class
    // The HD Hex Motor has 56 ticks per revolution
    //    the 20:1 is geared 20 to 1
    //    the 40:1 is geared 40 to 1
    // The HD Hex Motor is also used with the Ultraplanetary gearbox
    //    the 3:1 cartridge is actually 84:29 (2.9...)
    //    the 4:1 cartridge is actually 76:21 (3.6...)
    //    the 5:1 cartridge is actually 68:13 (5.2...)
    static final double HEX_HD_RATIO_3_1 = 84.0/29.0;
    static final double HEX_HD_RATIO_4_1 = 76.0/21.0;
    static final double HEX_HD_RATIO_5_1 = 68.0/13.0;
    static final double COUNTS_PER_REV = 56.0 * HEX_HD_RATIO_4_1 * HEX_HD_RATIO_5_1;

    //245 counts ~ 90 degree turn

    //static final double COUNTS_PER_INCH = (288)/(3.54331*(Math.PI)); //for core hex motor
    static final double COUNTS_PER_INCH = (COUNTS_PER_REV)/(3.54331*(Math.PI)*2);

    static final double TURN_VALUE_90 = 532/(COUNTS_PER_INCH); //BEFORE: 254, left 495, right -569 turn right, avg532

    //Detection Constants, 0, 1, or 4 rings in stack

    //test path
    static final SCHSPathSeg[] testPathRun = {
            new SCHSPathSeg(12, 12, 1.0),  // Forward 1 ft
            new SCHSPathSeg(-TURN_VALUE_90, TURN_VALUE_90, 1.0),  // Left 90
            new SCHSPathSeg(24, 24, 1.0),  // Forward 2 ft
            new SCHSPathSeg(TURN_VALUE_90, -TURN_VALUE_90, 1.0),  // right 90
            new SCHSPathSeg(24, 24, 1.0),  // Forward 2 ft
    };

    static final SCHSPathSeg[] moveToShoot = {
            new SCHSPathSeg(48, 48, 0.9)
    };

    //go to target zone A to deposit wobble goal (0 rings)
    static final SCHSPathSeg[] goToTargetA = {
            new SCHSPathSeg(17, 17, 0.9),  // Forward 13.5 in
            new SCHSPathSeg(-TURN_VALUE_90, TURN_VALUE_90, 0.9),  // Right 90
            //new SCHSPathSeg(6, 6, 0.9),  // Forward 6 in
    };

    static final SCHSPathSeg[] goToLaunchA = {
            new SCHSPathSeg(-6, -6, 0.9),  // backward 6 in
    };

    //go to target zone B to deposit wobble goal (1 rings)
    static final SCHSPathSeg[] goToTargetB = {
            new SCHSPathSeg(16, 16, 0.9),  // Forward 22.5 in
    };

    //go to launch line from target zone B
    static final SCHSPathSeg[] goToLaunchB = {
            new SCHSPathSeg(-2, -2, 0.9),  // Back 6 in
    };

    //go to target zone C to deposit wobble goal (4 rings)
    static final SCHSPathSeg[] goToTargetC = {
            new SCHSPathSeg(65, 65, 0.9),  // Forward 62.5 in
            new SCHSPathSeg(-TURN_VALUE_90, TURN_VALUE_90, 0.9),  // Right 90
            //new SCHSPathSeg(6, 6, 0.9),  // Forward 6 in
    };

    //go to launch line from target zone C
    static final SCHSPathSeg[] goToLaunchC = {
            new SCHSPathSeg(-6, -6, 0.9),  // Back 6in
            new SCHSPathSeg(TURN_VALUE_90, -TURN_VALUE_90, 0.9),  // left 90
            new SCHSPathSeg(-48, -48, 0.9),  // Back 4 ft
    };

    //New paths for shooting from new location, with pidf control, -18 from original straight path
    static final SCHSPathSeg[] moveToShootNew = {
            new SCHSPathSeg(26, 26, 0.9) //forward 30 in (-18in from original path)
    };

    //go to target zone A to deposit wobble goal (0 rings)
    static final SCHSPathSeg[] goToTargetANew = {
            new SCHSPathSeg(39, 39, 0.9),  // Forward 13.5  (+18in from original path)
            new SCHSPathSeg(-TURN_VALUE_90, TURN_VALUE_90, 0.9),  // Right 90
            //new SCHSPathSeg(6, 6, 0.9),  // Forward 6 in
    };

    static final SCHSPathSeg[] goToLaunchANew = {
            new SCHSPathSeg(-6, -6, 0.9),  // backward 6 in
    };

    //go to target zone B to deposit wobble goal (1 rings)
    static final SCHSPathSeg[] goToTargetBNew = {
            new SCHSPathSeg(38, 38, 0.9),  // Forward 22.5 in (+18in from original path)
    };

    //go to launch line from target zone B
    static final SCHSPathSeg[] goToLaunchBNew = {
            new SCHSPathSeg(-1, -1, 0.9),  // Back 6 in
    };

    //go to target zone C to deposit wobble goal (4 rings)
    static final SCHSPathSeg[] goToTargetCNew = {
            new SCHSPathSeg(87, 87, 0.9),  // Forward 62.5 in (+18in from original path)
            new SCHSPathSeg(-TURN_VALUE_90, TURN_VALUE_90, 0.9),  // Right 90
            //new SCHSPathSeg(6, 6, 0.9),  // Forward 6 in
    };

    //go to launch line from target zone C
    static final SCHSPathSeg[] goToLaunchCNew = {
            new SCHSPathSeg(-6, -6, 0.9),  // Back 6in
            new SCHSPathSeg(TURN_VALUE_90, -TURN_VALUE_90, 0.9),  // left 90
            new SCHSPathSeg(-48, -48, 0.9),  // Back 4 ft
    };

    static final SCHSPathSeg[] testA = {
            new SCHSPathSeg(12, 12, 0.9)
    };

    static final SCHSPathSeg[] testB = {
            new SCHSPathSeg(24, 24, 0.9)
    };

    static final SCHSPathSeg[] testC = {
            new SCHSPathSeg(36, 36, 0.9)
    };

    //Tensor Flow Object detection
    static final String VUFORIA_KEY = "AUnX7nP/////AAABmZjfOTd2skx4p/r+LBA29VQAFar5mbPnEfGtcl78mMIqK+EtsUOR33zwyiDCmj1oYMUx0P4eWZGi6EMhZgTM66/5llx5azKwGGxGmTJUGotbAekyZgxYR7SWDme6xMYGR68jZcR9rkvJxfB1ZKFytPXWeRpwzSAQJ0VACF/hdguUyfA6SSkF2dnc/iH76TkSV3hA4zz0v3wjHfQmmNBvrtgPklvfOTX2f+G5tBfBq75PEx52LaX+tOPTtBajR9MFwVT26kcqFz2GJCEBgjO3PX1St0xNJBqbbudKvZ+B/6xWuVhwHVqwOgy/RsuHLBFskh4n9Ec1xnuB9uCnQXrrliEtcR1TbnmIEYTX6FZtxF5H";
    static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    static final int SCAN_TIME = 3000;
    static final String LABEL_FIRST_ELEMENT = "Quad";
    static final String LABEL_SECOND_ELEMENT = "Single";

    //core hex motor constants
    static final double CHMOTOR_COUNTS_PER_REVOLUTION = 288;
    static final double REAR_WHEEL_BASE_= 12; //inches
    static final double TRACTION_WHEEL_DIAMETER = 90 * 0.0393701; //90mm converted to inches

    //formula for inches to counts (encoder value): 288 counts/revolution
    //1 revolution = 90pi mm = 3.54331pi inches
    //total counts = 288*rev
    //x inches / 3.54331pi = # rev
    //encoder value = (288*x)/(3.54331pi) = 310.466 at x = 12 inches

    //formula for degrees (encoder value): (a/360)*(3.54331pi) = y inches
    //encoder value = (288*y)/(12pi) = 310.466 at y inches for a degrees
    //at a = 90 degrees, encoder value = 243.839

}
