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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.Motion.*;

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

    // Robot voltage
    private double voltage;

    // drive motors
    private DcMotorEx dcmotorLeft = null;
    private DcMotorEx dcmotorRight = null;

    // drive mode: true is POVMode, false is TankMode
    private boolean boolPOVDrive = true;

    private DcMotorEx dcmotorFlail = null;

    // imu / gyro
    // in previous years, the gyro was unstable: it would cause some inits to hang
    // For Ultimate Goal, it is supposedly much more stable.
    // I have had the imu fail to initialize.
    private BNO055IMU imu;
    // State used for updating telemetry
    private Orientation angles;
    // boolGravity == true -> take and report gravity measurements
    private boolean boolGravity = true;
    private Acceleration gravity;
    AxesOrder axesorder = AxesOrder.ZYX;

    // Some Game parameters...

    // Games are Traditional or Remote
    //   that distinction affects field size and mid-goal shooting

    // which alliance I'm on
    enum Alliance {RED, BLUE}
    private Alliance alliance = Alliance.BLUE;
    private String startLine = "right";

    // The game has 3 TargetZones
    enum TargetZone {A, B, C}
    // TensorFlow should figure out the correct TargetZone
    // 0 rings = LandingZone.A, 1 ring = LandingZone.B, 4 rings LandingZone.C
    private TargetZone targetZone = TargetZone.C;

    private String[] astrRoute = {"r1", "r2", "r3", "r4"};
    private int cRoute = astrRoute.length;
    int iRoute = 0;
    boolean bRouteChanging = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Init demo start");

        voltage = getBatteryVoltage();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        dcmotorLeft = hardwareMap.get(DcMotorEx.class, "leftMotor");
        dcmotorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        dcmotorLeft.setDirection(DcMotor.Direction.REVERSE);
        dcmotorRight.setDirection(DcMotor.Direction.FORWARD);

        // After the drive motors are configured, inform the Motion class
        // odometry
        setRobot(dcmotorLeft, dcmotorRight);
        // use an old robot
        // TODO: use a phantom switch to determine the actual robot
        setRobot2019();


        // flail motor hack
        // TODO: hack!
        dcmotorFlail = hardwareMap.get(DcMotorEx.class, "motorWobble");
        dcmotorFlail.setDirection(DcMotorSimple.Direction.REVERSE);


        // TODO: this is a gyro parameter for the 2019 robot
        // the AxesOrder for the 2019 robot.
        axesorder = AxesOrder.ZXY;

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

        // TODO: Initialize Tensor Flow

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Init demo done");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // Update the pose
        Motion.updateRobotPose();

        // look at the imu
        if (imu.isGyroCalibrated()) {
            telemetry.addData("IMU", "calibrated");
        } else {
            telemetry.addData("IMU", "calibrating");
        }

        // TODO: check other health issues
        telemetry.addData("Health", "Battery %.2f", voltage);

        // process some simple commands to configure the robot
        if (gamepad1.x) {
            // x is a blue button
            alliance = Alliance.BLUE;
        }
        if (gamepad1.b) {
            // b is a red button
            alliance = Alliance.RED;
        }
        telemetry.addData("Alliance", alliance);

        if (gamepad1.dpad_left) {
            startLine = "left";
        }
        if (gamepad1.dpad_right) {
            startLine = "right";
        }
        telemetry.addData("StartLine", startLine);

        // non idempotent buttons...
        // we need to be careful here.
        // Notice the initial button press and act
        //   but ignore all subsequent times the button is still active
        // TODO: a better way would be to use gamepad1Copy.copy(gamepad1)
        //   fields in a Gamepad may be read or written
        if (!bRouteChanging && gamepad1.dpad_up) {
            // increment the route
            iRoute = (iRoute + 1) % cRoute;
            bRouteChanging = true;
        }
        if (!bRouteChanging && gamepad1.dpad_down) {
            // decrement the route
            iRoute = (iRoute + cRoute - 1) % cRoute;
            bRouteChanging = true;
        }
        if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            bRouteChanging = false;
        }
        telemetry.addData("Route", astrRoute[iRoute]);

        // TODO: use Tensor Flow to determine height of the stack.
        // The height might vary over time. It should start at "quad".

        telemetry.addData("TargetZone", targetZone);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        Motion.updateRobotPose();
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

        // process some simple commands
        // reset the pose
        if (gamepad1.y) {
            setPoseInches(0, 0, 0);
        }

        // Send calculated power to wheels
        dcmotorLeft.setPower(powerLeft);
        dcmotorRight.setPower(powerRight);

        // move the flails
        // TODO: assign motor to flails. Core Hex is not working; will try an HD
        dcmotorFlail.setPower(gamepad1.right_trigger);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", powerLeft, powerRight);

        // report the current pose
        telemetry.addData("Pose", "x %8.2f %8.2f %8.2f", xPose, yPose, thetaPoseDegrees);

        // query the imu
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, axesorder, AngleUnit.DEGREES);


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

        // if we are checking the accelerometers
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
