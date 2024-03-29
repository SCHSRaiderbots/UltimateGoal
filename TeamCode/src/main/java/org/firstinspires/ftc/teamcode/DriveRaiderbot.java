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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DriveRaiderbot", group="SCHS")
//@Disabled
public class DriveRaiderbot extends OpMode {


    //This OpMode will be used to begin testing all components together (with driver control) as they are built
    //Currently includes ability to drive robot chassis and shooter with 2 motors

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx shooterMotor1 = null;
    private DcMotorEx shooterMotor2 = null;
    private Servo shooterServo = null;

    private DcMotorEx leftMotor = null;
    private DcMotorEx rightMotor = null;

    private DcMotorEx motorIntake = null;
    //private DcMotorEx shooterMotor1 = null;
    //private DcMotorEx shooterMotor2 = null;
    private boolean isShooterOn = false;

    private Servo grabberServo = null;
    private DcMotorEx grabberMotor = null;

    private final int creepConstant = 904;
    private int[] creepArray = new int[2];
    private double driveMultiplier = 0.75;

    //Creep methods and helper methods - creep up/around slowly with d-pad
    private void creep() {

        int currentPosLeft = leftMotor.getCurrentPosition();
        int currentPosRight = rightMotor.getCurrentPosition();

        setCreepArray(0, 0); //changed to be similar to old creep method which reset targets to 0 every time it ran

        //DEFAULT ELSE WILL NEVER BE CALLED BECAUSE THE METHOD IS ONLY EVER CALLED WHENEVER THE DPAD IS PRESSED
        if (gamepad1.dpad_left)
            creepLeft(currentPosLeft, currentPosRight);
        else if (gamepad1.dpad_right)
            creepRight(currentPosLeft, currentPosRight);
        else if (gamepad1.dpad_up)
            creepForward(currentPosLeft, currentPosRight);
        else if (gamepad1.dpad_down)
            creepBack(currentPosLeft, currentPosRight);



        leftMotor.setTargetPosition(getCreepArrayLeftPosition());
        rightMotor.setTargetPosition(getCreepArrayRightPosition());

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(0.1);
        rightMotor.setPower(0.1);
    }

    private void creepBack(int leftPos, int rightPos) {

        int targetPosLeft = leftPos - creepConstant;
        int targetPosRight = rightPos - creepConstant;

        setCreepArray(targetPosLeft, targetPosRight);

    }

    private void creepForward(int leftPos, int rightPos) {

        int targetPosLeft = leftPos + creepConstant;
        int targetPosRight = rightPos + creepConstant;

        setCreepArray(targetPosLeft, targetPosRight);

    }

    private void creepLeft(int leftPos, int rightPos) {

        int targetPosLeft = leftPos - creepConstant;
        int targetPosRight = rightPos + creepConstant;

        setCreepArray(targetPosLeft, targetPosRight);

    }

    private void creepRight(int leftPos, int rightPos) {

        int targetPosLeft = leftPos + creepConstant;
        int targetPosRight = rightPos - creepConstant;

        setCreepArray(targetPosLeft, targetPosRight);

    }

    private void setCreepArray(int leftTarget, int rightTarget) {

        creepArray[0] = leftTarget;
        creepArray[1] = rightTarget;

    }

    private int getCreepArrayLeftPosition() {

        return creepArray[0];

    }

    private int getCreepArrayRightPosition() {

        return creepArray[1];

    }

    @Override
    public void init() {

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        leftMotor  = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
        motorIntake = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        grabberServo = hardwareMap.get(Servo.class, "grabberServo");
        grabberMotor = hardwareMap.get(DcMotorEx.class, "grabberMotor");

        // Needs one side to be reversed and one side to be forward so that ring shoots out in correct direction
        // Wheels are on either side of ring, so reverse one motor to run backwards to allow ring to launch forward
        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.REVERSE);

        // assume max velocity of 90 rps gives 2^15 PWM
        // changing this does not have much effect
        double Fshoot = 32767.0 / (90.0 * 28.0);

        // 10, 3 gives 6 seconds with ringing
        // 50, 5 gives 3 seconds with overshoot
        // 20, 2 gives 4 seconds with overshoot
        // 50, 2 gives 6 seconds
        // 50, 15 gives 4 seconds
        PIDFCoefficients pidfRUEshoot = new PIDFCoefficients(50.0, 5, 0, Fshoot, MotorControlAlgorithm.PIDF);
        // shooter does not use R2P...
        // PIDFCoefficients pidfR2P = new PIDFCoefficients(10, 0.05, 0, 0, MotorControlAlgorithm.PIDF);

        // set the PIDF coefficients
        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUEshoot);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUEshoot);

        shooterMotor1.setVelocity(0.0);
        shooterMotor2.setVelocity(0.0);

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // describe the motor...
        // Default PIDF:
        //   PIDF(rue) 10, 3, 0, 0, Legacy
        //   PIDF(r2p) 10, 0.05, 0, 0, Legacy
        LogDevice.dump("shooter1", shooterMotor1);
        LogDevice.dump("shooter2", shooterMotor2);


        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        double F = 32767.0 / (2.0 * 288.0);
        PIDFCoefficients pidfRUE = new PIDFCoefficients(10.0, 1.0, 0.0, F, MotorControlAlgorithm.PIDF );
        PIDFCoefficients pidfR2P = new PIDFCoefficients(10.0, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);
        grabberMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUE);
        grabberMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfR2P);
        grabberMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        grabberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabberMotor.setTargetPosition(0);
        grabberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabberMotor.setPower(1.0);

        // Wait for the game to start (driver presses PLAY)
        runtime.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            driveMultiplier = 1;
        } else if (!gamepad1.a) {
            driveMultiplier = 0.5;
        }

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = driveMultiplier * (Range.clip(drive + turn, -1.0, 1.0)) ;
        rightPower   = driveMultiplier * (Range.clip(drive - turn, -1.0, 1.0)) ;

            /*if (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right) {
                creep();
            }*/

        //L2 = quarter turn, wobble goal arm straight up
        //R2 = wobble goal arm straight out right

        //int wobbleTargetOut = (int) (gamepad2.left_trigger * 288);
        int wobbleTargetOut = (int) (gamepad2.left_trigger * 280) + 60;
        grabberMotor.setTargetPosition(wobbleTargetOut);
        grabberMotor.setPower(0.8);

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;
//Input velocity from gamepad joystick
        double velocity = gamepad2.left_stick_y * 950;
        velocity = Range.clip(velocity, -950, 950) ;

        // Send calculated velocity to shooter motors
        shooterMotor1.setVelocity(velocity);
        shooterMotor2.setVelocity(velocity - 100);

        /*if (gamepad2.a) {
            isShooterOn = true;
        }
        if (gamepad2.b) {
            isShooterOn = false;
        }

        if (isShooterOn) {
            shooterMotor1.setVelocity(1000);
            shooterMotor2.setVelocity(1000-100);
        } else if (!isShooterOn) {
            shooterMotor1.setVelocity(0);
            shooterMotor2.setVelocity(0);
        }*/



       /* //full blast (highest speed and distance) - velocity ~ 2000 ticks/sec
        double BLAST_SPEED = 2000;
        double HALF_BLAST = 1500;
        if (gamepad2.a) {
            shooterMotor1.setVelocity(BLAST_SPEED);
            shooterMotor2.setVelocity(BLAST_SPEED - 100);

        }

        if (gamepad2.b) {
            shooterMotor1.setVelocity(HALF_BLAST);
            shooterMotor2.setVelocity(HALF_BLAST);
        }*/

        shooterServo.setDirection(Servo.Direction.FORWARD);
        //open and close grabber servo with L! and R1 bumper buttons on gamepad2
            /*if (gamepad2.left_bumper) {
                shooterServo.setPosition(0);
            } else if (gamepad2.right_bumper) {
                shooterServo.setPosition(0.2);
            }/*

            */

        if (gamepad2.right_bumper) {
            shooterServo.setPosition(0.2);
        } else if (!gamepad2.right_bumper) {
            shooterServo.setPosition(0);
        }

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        motorIntake.setPower(gamepad1.right_trigger);

        //open and close grabber servo with x and y buttons on gamepad2
        if (gamepad2.x) {
            grabberServo.setPosition(0);
        } else if (gamepad2.y) {
            grabberServo.setPosition(0.4);
        }

        /*
        //turn wobble goal arm (grabberMotor) by moving left joystick up and down on gamepad2
        double grabberPower = -gamepad2.right_stick_y;




        grabberPower = Range.clip(grabberPower, -0.2, 0.2);
        grabberMotor.setPower(grabberPower*0.8);
         */
        // Show the elapsed game time and wheel power and shooter motor velocity (input and actual)

        telemetry.addData("Status", "grabber position:" + grabberMotor.getCurrentPosition());
        telemetry.addData("Status", "Motor velocity input: " + velocity);
        telemetry.addData("Status", "Shooter Motor 1 velocity actual (tick/sec): " + shooterMotor1.getVelocity());
        telemetry.addData("Status", "Shooter Motor 2 velocity actual (tick/sec): " + shooterMotor2.getVelocity());

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Status", "Grabber Servo Position: " + grabberServo.getPosition());
        telemetry.update();

            //turn wobble goal arm (grabberMotor) by moving left joystick up and down on gamepad

            // Show the elapsed game time and wheel power and shooter motor velocity (input and actual)
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Status", "Motor velocity input (radians): " + velocity);
            //telemetry.addData("Status", "Motor velocity actual (radians): " + shooterMotor.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("Status", "Shooter Motor 1 velocity actual (tick/sec): " + shooterMotor1.getVelocity());
            telemetry.addData("Status", "Shooter Motor 2 velocity actual (tick/sec): " + shooterMotor2.getVelocity());
            telemetry.addData("Status", "Grabber Servo Position: " + grabberServo.getPosition());
            telemetry.update();
        }
    }

