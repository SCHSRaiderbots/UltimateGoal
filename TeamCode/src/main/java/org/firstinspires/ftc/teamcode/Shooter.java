package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Basic: Shooter", group="Iterative Opmode")
public class Shooter extends OpMode {

    // the shooter motor
    DcMotorEx dcmotorShooter = null;

    // whether the shooter motor is on
    boolean bShooter = false;

    Servo servoShoot = null;

    @Override
    public void init() {

        // find the shooter motor
        // tryGet() method is used so failure to find the motor will not throw an exception
        // TODO: investigate the type of motoer and its tick count
        // I believe it is listed as a VersaPlanetary, but no gear ratio is specifified
        // use RUN_TO_POSITION to figure out encoder counts
        dcmotorShooter = hardwareMap.tryGet(DcMotorEx.class, "motorShooter");

        if (dcmotorShooter != null) {
            // set reverse direction
            dcmotorShooter.setDirection(DcMotor.Direction.REVERSE);

            // describe the motor...
            LogDevice.dump("shooter", dcmotorShooter);
        }

        servoShoot = hardwareMap.tryGet(Servo.class, "servoShoot");

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        if (servoShoot != null) {
            // TODO: clip to [0,1]
            servoShoot.setPosition(gamepad1.right_stick_y);
            telemetry.addData("servo", "servopos %f", gamepad1.right_stick_y);
        } else {
            telemetry.addData("servo", "no shooter servo");
        }

        // only process the shooter motor if one was found...
        if (dcmotorShooter != null) {

            // if the shooter is off and the driver presses a button
            if (!bShooter && gamepad1.a) {
                // remember the shooter is on
                bShooter = true;

                // turn on the motor
                // we'll use setPower() to start, but PID velocity control would be better
                // dcmotorShooter.setPower(0.5);

                // try controlling the velocity
                dcmotorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // trying to get 1 rps, but failing
                // spins way too fast.
                // try fudge factor of 20
                //   1 rps -- intermittent motion! (need to adjust PIDF?) (Battery was also low)
                //   10 rps -- better control, but will not shoot
                //   50 rps -- marginal shooter; reports 1.78/3.3
                //  100 rps -- maxed out velocity
                dcmotorShooter.setVelocity(360.0 * 50.0 / 20.0, AngleUnit.DEGREES);
                // dcmotorShooter.setVelocity(28);
            }

            if (!bShooter && gamepad1.x) {
                // remember the shooter is on
                bShooter = true;

                // turn on the motor
                dcmotorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                dcmotorShooter.setVelocity(60.0 * 28.0);
                // dcmotorShooter.setPower(1.0);
                // reads 84.0 rps when I expected 60.
            }

            if (!bShooter && gamepad1.y) {
                // remember the shooter is on
                bShooter = true;

                // turn on the motor
                // we'll use setPower() to start, but PID velocity control would be better
                // 3.2 rps reported, but that is suspect
                dcmotorShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                dcmotorShooter.setPower(1.0);
            }

            // if the driver presses the b button, turn off the shooter
            if (gamepad1.b) {
                // remember the shooter is off
                bShooter = false;

                // turn off the motor
                dcmotorShooter.setPower(0.0);

                // restore default run mode
                dcmotorShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // report the motor velocity
            // TODO: these numbers do not make sense yet
            //   OK, starting to make sense; motor presumes divide by 20.
            // try revs per second
            telemetry.addData("Shooter", "vel %f", 20.0 * dcmotorShooter.getVelocity(AngleUnit.DEGREES)/360.0);
            // use known ticks to get a value
            // expect velocity to be in ticks per second; dividing by 28 should be rotations per second
            // press x at 11.0 V, get 97 and 69.
            //   took out .setPower(1.0), got 84 and 60 (I had set 60).
            //   so take 84/28 = 3
            // press y, get 97 and 69, so those are flat out values for 11.40 V.
            telemetry.addData("Shooter (from ticks)", dcmotorShooter.getVelocity()/28.0);
            telemetry.addData("Shooter velocity", "vel %5.2f m/s", Math.PI * 0.090 * dcmotorShooter.getVelocity()/28.);
        }
        else {
            telemetry.addData("Shooter", "not present");
        }

    }

    @Override
    public void stop() {

    }
}
