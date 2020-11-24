/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Demo", group="Iterative Opmode")
public class Demo extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // drive motors
    private DcMotorEx dcmotorLeft = null;
    private DcMotorEx dcmotorRight = null;

    // drive mode: true is POVMode, false is TankMode
    private boolean boolPOVDrive = true;

    // gyro
    // in previous years, the gyro was unstable: it would cause some inits to hang
    // For Ultimate Goal, it is supposedly much more stable.
    // I have had the imu fail to initialize.
    private BNO055IMU imu;
    // State used for updating telemetry
    private Orientation angles;
    // boolGravity == true -> take and report gravity measurements
    private boolean boolGravity = true;
    private Acceleration gravity;


    // Info for the robot pose
    // TODO: abstract to an Odometry class

    // robot parameters
    // the wheel diameters are 90mm nominal
    private double mWheelDiameterLeft = 0.090;
    private double mWheelDiameterRight = 0.090;

    // half the distance between the wheels
    // the new wheel separation 13 + 15/16
    double distWheel = (14.0 - (1.0/16.0)) * 0.0254 / 2;

    // The CoreHex motor has 4 ticks per revolution and is geared down by 72
    //   those attributes should be in the DcMotor class

    // The HD Hex Motor has 56 ticks per revolution
    //    or so claims http://www.revrobotics.com/content/docs/HDMotorEncoderGuide.pdf
    //    the 20:1 is geared 20 to 1
    //    the 40:1 is geared 40 to 1
    // The HD Hex Motor is also used with the Ultraplanetary cartridges
    //    the 3:1 cartridge is actually 84:29 (2.9...)
    //    the 4:1 cartridge is actually 76:21 (3.6...)
    //    the 5:1 cartridge is actually 68:13 (5.2...)
    private final double HD_HEX_GEAR_CART_3_1 = 84.0/29.0;
    private final double HD_HEX_GEAR_CART_4_1 = 76.0/21.0;
    private final double HD_HEX_GEAR_CART_5_1 = 68.0/13.0;

    // calculate the wheel's ticks per revolution
    //   Apparently, we only see 1/2 the counts that REV claims (28 instead of 56)
    double ticksPerWheelRev = (56.0/2.0) * HD_HEX_GEAR_CART_5_1 * HD_HEX_GEAR_CART_4_1;

    // derived robot parameters
    // Distance per tick
    //   leaving the units vague at this point

    // the distance per tick for each wheel = circumference / ticks
    private double distpertickLeft = mWheelDiameterLeft * Math.PI / (ticksPerWheelRev);
    private double distpertickRight = mWheelDiameterRight * Math.PI / (ticksPerWheelRev);

    // the robot pose
    //   can have .updatePose(), .getPose(), .setPose()
    //   using static should allow the Pose to be carried over from Autonomous to Teleop
    //     Autonomous can set the initial pose
    //     When Teleop starts, it can use the existing Pose
    //        If there was no autonomous, then initial Pose is random
    //        A button press during teleop's init_loop could set a known Pose
    static double xPose = 0.0;
    static double yPose = 0.0;
    static double thetaPose = 0.0;

    // this shadow state need not be static
    double xPoseInches = 0.0;
    double yPoseInches = 0.0;
    double thetaPoseDegrees = 0.0;

    // encoder counts
    // There's a subtle issue here
    //    If robot is not moving, it is OK to set these values to the current encoder counts
    //    That could always happen during .init()
    private int cEncoderLeft;
    private int cEncoderRight;

    /**
     * Want to use this code with last year's robot, but it has different parameters
     */
    void setRobot2019() {
        // set the wheel diameters
        mWheelDiameterLeft = 0.090;
        mWheelDiameterRight = 0.090;

        // set the wheel half separation
        distWheel =  (0.305 / 2) * 360.0 / 362.0;

        // ticks per wheel revolution
        // CoreHex motor...
        ticksPerWheelRev = 4 * 72;

        // derived values
        distpertickLeft = mWheelDiameterLeft * Math.PI / (ticksPerWheelRev);
        distpertickRight = mWheelDiameterRight * Math.PI / (ticksPerWheelRev);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Init demo start");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        dcmotorLeft = hardwareMap.get(DcMotorEx.class, "leftMotor");
        dcmotorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        dcmotorLeft.setDirection(DcMotor.Direction.REVERSE);
        dcmotorRight.setDirection(DcMotor.Direction.FORWARD);

        // odometry
        setRobot2019();
        // remember the current encoder counts to do odometry
        // DcMotor Direction also affects the encoder counts
        // remember the current encoder counts
        // Should always do this (even if not resetting the Pose)
        cEncoderLeft = dcmotorLeft.getCurrentPosition();
        cEncoderRight = dcmotorRight.getCurrentPosition();



        // grab the imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        // set typical parameters
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        // deal with these parameters later
        // parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        // parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // start initializing the IMU
        telemetry.addData("IMU", "initialize");
        imu.initialize(parameters);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Init demo done");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        // look at the imu
        if (imu.isGyroCalibrated()) {
            telemetry.addData("IMU", "calibrated");
        } else {
            telemetry.addData("IMU", "calibrating");
        }

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double powerLeft;
        double powerRight;

        // odometry
        updateRobotPose();

        // Choose to drive using either Tank Mode, or POV Mode
        if (boolPOVDrive) {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            powerLeft = Range.clip(drive + turn, -1.0, 1.0);
            powerRight = Range.clip(drive - turn, -1.0, 1.0);
        }
        else {
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            powerLeft  = -gamepad1.left_stick_y ;
            powerRight = -gamepad1.right_stick_y ;
        }

        // Send calculated power to wheels
        dcmotorLeft.setPower(powerLeft);
        dcmotorRight.setPower(powerRight);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", powerLeft, powerRight);

        // report the current pose
        // TODO: angle OK; distance is off
        telemetry.addData("Pose", "x %8.2f %8.2f %8.2f", xPose, yPose, thetaPoseDegrees);

        // query the imu
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // telemetry.addData("imu status", imu.getSystemStatus().toShortString());
        // telemetry.addData("imu calib", imu.getCalibrationStatus().toString());

        // heading, pitch, and roll depend on how the Expansion Hub is installed on the robot.
        // So this choice depends on the robot.
        // If the EH is flat, then the firstAngle is the heading
        // If flat and the short axis is forward, then second Angle is the pitch
        telemetry.addData("imu", "heading %.1f roll %.1f pitch %.1f",
                angles.firstAngle,
                angles.secondAngle,
                angles.thirdAngle);

        if (boolGravity) {
            // get the gravity measurements
            gravity  = imu.getGravity();

            // report the gravity measurements
            telemetry.addData("gravity", gravity.toString());
            telemetry.addData("gravmag", "%8.3f m/s/s",
                    Math.sqrt(gravity.xAccel * gravity.xAccel +
                            gravity.yAccel * gravity.yAccel +
                            gravity.zAccel * gravity.zAccel));
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //
    }


    /**
     * Update the robot pose.
     * Uses small angle approximations.
     * See COS495-Odometry by Chris Clark, 2011,
     * <a href="https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf">https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf</a>
     */
    private void updateRobotPose() {
        // several calculations are needed

        // get the new encoder positions
        int cLeft = dcmotorLeft.getCurrentPosition();
        int cRight = dcmotorRight.getCurrentPosition();

        // calculate the arc length deltas
        int dsLeft = cLeft - cEncoderLeft;
        int dsRight = cRight - cEncoderRight;

        // save the new encoder positions for the next time around
        cEncoderLeft = cLeft;
        cEncoderRight = cRight;

        // calculate the distance the wheels moved
        double distL = dsLeft * distpertickLeft;
        double distR = dsRight * distpertickRight;

        // approximate the arc length as the average of the left and right arcs
        double ds = (distR + distL) / 2;
        // approximate the angular change as the difference in the arcs divided by wheel offset from
        // center of rotation.
        double dtheta = (distR - distL) / ( 2 * distWheel);

        // approximate the hypotenuse as just ds
        // approximate the average change in direction as one half the total angular change
        double dx = ds * Math.cos(thetaPose + 0.5 * dtheta);
        double dy = ds * Math.sin(thetaPose + 0.5 * dtheta);

        // update the current pose
        xPose = xPose + dx;
        yPose = yPose + dy;
        thetaPose = thetaPose + dtheta;

        // convert to inches and degrees
        xPoseInches = xPose / 0.0254;
        yPoseInches = yPose / 0.0254;
        thetaPoseDegrees = thetaPose * (180.0 / Math.PI);
    }

    void setPoseInches(double x, double y, double theta) {
        // convert to meters and set state variables
        xPose = x * 0.0254;
        yPose = y * 0.0254;
        thetaPose = theta * (Math.PI / 180.0);

        // copy to imperial to shadow state for consistency
        xPoseInches = x;
        yPoseInches = y;
        thetaPoseDegrees = theta;
    }

    /**
     * Read the battery voltage from all available voltage sensors
     * @return the minimum battery voltage or positive infinity
     */
    private double getBatteryVoltage() {
        // set an infinite voltage
        double result = Double.POSITIVE_INFINITY;

        // examine each voltage sensors
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            // get the voltage reading for that sensor
            double voltage = sensor.getVoltage();

            // if the voltage is reasonable
            if (voltage > 0) {
                // then accumulate the result
                result = Math.min(result, voltage);
            }
        }

        // return the minimum voltage
        return result;
    }


}
