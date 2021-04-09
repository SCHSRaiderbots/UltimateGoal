package org.firstinspires.ftc.teamcode;

class SCHSConstants {

    // power constants between 1 and -1
    static final double POWER_FULL = 1;
    static final double POWER_HALF = 0.5;
    static final double POWER_TURN_SPEED = 0.5;

    //Robot Constants
    static final double ROBOT_WIDTH = 14; //inches
    static final double ROBOT_MAX_SPEED = 4; //feet/sec
    static final int DRIVE = 0;
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

    //go to foundation
    static final SCHSPathSeg[] goToFDPath = {
            new SCHSPathSeg(6, 6, 0.9),  // Forward 3.5 in
            new SCHSPathSeg(-TURN_VALUE_90, TURN_VALUE_90, 0.9),  // Left 90
            new SCHSPathSeg(24, 24, 0.9),  // Forward 2 ft
            new SCHSPathSeg(TURN_VALUE_90, -TURN_VALUE_90, 0.9),  // right 90
            new SCHSPathSeg(17, 17, 0.9),  // Forward 15
    };

    //test path
    static final SCHSPathSeg[] testPathRun = {
            new SCHSPathSeg(12, 12, 0.75),  // Forward 1 ft
            new SCHSPathSeg(-TURN_VALUE_90, TURN_VALUE_90, 0.75),  // Left 90
            new SCHSPathSeg(24, 24, 0.75),  // Forward 2 ft
            new SCHSPathSeg(TURN_VALUE_90, -TURN_VALUE_90, 0.75),  // right 90
            new SCHSPathSeg(24, 24, 0.75),  // Forward 2 ft
    };

    //Tensor Flow Object detection
    static final String VUFORIA_KEY = "";
    static final String TFOD_MODEL_ASSET = "";

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
