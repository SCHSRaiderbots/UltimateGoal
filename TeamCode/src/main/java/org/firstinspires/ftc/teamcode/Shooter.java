package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Basic: Shooter", group="Iterative Opmode")
public class Shooter extends OpMode {

    // the shooter motor
    DcMotorEx dcmotorShooter = null;

    // whether the shooter motor is on
    boolean bShooter = false;

    @Override
    public void init() {

        // find the shooter motor
        // tryGet() method is used so failure to find the motor will not throw an exception
        dcmotorShooter = hardwareMap.tryGet(DcMotorEx.class, "motorShooter");

        if (dcmotorShooter != null) {
            // set reverse direction
            dcmotorShooter.setDirection(DcMotor.Direction.REVERSE);
        }

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        // only process the shooter motor if one was found...
        if (dcmotorShooter != null) {

            // if the shooter is off and the driver presses a button
            if (!bShooter && gamepad1.a) {
                // remember the shooter is on
                bShooter = true;

                // turn on the motor
                // we'll use setPower() to start, but PID velocity control would be better
                dcmotorShooter.setPower(0.5);
            }

            if (!bShooter && gamepad1.x) {
                // remember the shooter is on
                bShooter = true;

                // turn on the motor
                // we'll use setPower() to start, but PID velocity control would be better
                dcmotorShooter.setPower(0.75);
            }

            if (!bShooter && gamepad1.y) {
                // remember the shooter is on
                bShooter = true;

                // turn on the motor
                // we'll use setPower() to start, but PID velocity control would be better
                dcmotorShooter.setPower(1.0);
            }

            // if the driver presses a button
            if (gamepad1.b) {
                // remember the shooter is off
                bShooter = false;

                // turn off the motor
                dcmotorShooter.setPower(0.0);
            }

            // report the motor velocity
            telemetry.addData("Shooter", "vel %f", dcmotorShooter.getVelocity(AngleUnit.DEGREES));
        }
        else {
            telemetry.addData("Shooter", "not present");
        }

    }

    @Override
    public void stop() {

    }
}
