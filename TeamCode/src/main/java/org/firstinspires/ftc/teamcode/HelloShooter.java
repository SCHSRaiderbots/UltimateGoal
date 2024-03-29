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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


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

@TeleOp(name="HelloShooter", group="SCHS Test")
//@Disabled
public class HelloShooter extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx shooterMotor1 = null;
    private DcMotorEx shooterMotor2 = null;
    private Servo shooterServo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");

        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        // Needs one side to be reversed and one side to be forward so that ring shoots out in correct direction
        // Wheels are on either side of ring, so reverse one motor to run backwards to allow ring to launch forward
        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Input velocity from gamepad joystick
            double velocity = gamepad2.left_stick_y * 2000;
            velocity = Range.clip(velocity, -2000, 2000) ;

            // Send calculated velocity to shooter motors
            shooterMotor1.setVelocity(velocity);
            shooterMotor2.setVelocity(velocity);

            //full blast (highest speed and distance) - velocity ~ 2000 ticks/sec
            double BLAST_SPEED = 2000;
            double HALF_BLAST = 1000;
            if (gamepad2.a) {
                shooterMotor1.setVelocity(BLAST_SPEED);
                shooterMotor2.setVelocity(BLAST_SPEED);
            }

            if (gamepad2.b) {
                shooterMotor1.setVelocity(HALF_BLAST);
                shooterMotor2.setVelocity(HALF_BLAST);
            }

            shooterServo.setDirection(Servo.Direction.FORWARD);
            //open and close grabber servo with x and y buttons on gamepad2
            if (gamepad2.x) {
                shooterServo.setPosition(0);
            } else if (gamepad2.y) {
                shooterServo.setPosition(0.2);
            }

            // Show the elapsed game time and shooter motor velocity (input and actual).
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "Motor velocity input (radians): " + velocity);
            //telemetry.addData("Status", "Motor velocity actual (radians): " + shooterMotor.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("Status", "Shooter Motor 1 velocity actual (tick/sec): " + shooterMotor1.getVelocity());
            telemetry.addData("Status", "Shooter Motor 2 velocity actual (tick/sec): " + shooterMotor2.getVelocity());
            telemetry.update();
        }
    }
}
