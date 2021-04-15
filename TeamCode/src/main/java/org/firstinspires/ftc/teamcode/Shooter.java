package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Basic: Shooter", group="Iterative Opmode")
public class Shooter extends OpMode {

    // the shooter motor
    DcMotorEx dcmotorShooter = null;
    // whether the shooter motor is on
    boolean bShooter = false;

    Servo servoShoot = null;
    static final double posShooterRest = 0.0;
    static final double posShooterFire = 0.4;
    // current shooter position
    double posShooter = posShooterRest;

    @Override
    public void init() {

        // find the shooter motor
        // tryGet() method is used so failure to find the motor will not throw an exception
        dcmotorShooter = hardwareMap.tryGet(DcMotorEx.class, "motorShooter");

        // if there is a shooter motor, then initialize it
        if (dcmotorShooter != null) {
            // set reverse direction
            dcmotorShooter.setDirection(DcMotor.Direction.REVERSE);

            // assume max velocity of 90 rps gives 2^15 PWM
            // changing this does not have much effect
            double F = 32767.0 / (90.0 * 28.0);

            // 10, 3 gives 6 seconds with ringing
            // 50, 5 gives 3 seconds with overshoot
            // 20, 2 gives 4 seconds with overshoot
            // 50, 2 gives 6 seconds
            // 50, 15 gives 4 seconds
            PIDFCoefficients pidfRUE = new PIDFCoefficients(50.0, 5, 0, F, MotorControlAlgorithm.PIDF);
            // shooter does not use R2P...
            // PIDFCoefficients pidfR2P = new PIDFCoefficients(10, 0.05, 0, 0, MotorControlAlgorithm.PIDF);

            // set the PIDF coefficients
            dcmotorShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUE);

            // describe the motor...
            // Default PIDF:
            //   PIDF(rue) 10, 3, 0, 0, Legacy
            //   PIDF(r2p) 10, 0.05, 0, 0, Legacy
            LogDevice.dump("shooter", dcmotorShooter);
        }

        // initialize the servo that fires the ring
        servoShoot = hardwareMap.tryGet(Servo.class, "servoShoot");

        // if the shooter servo exists, command it to the rest position
        if (servoShoot != null) {
            // shooter at rest
            posShooter = posShooterRest;
            // command the servo to the current position
            servoShoot.setPosition(posShooter);
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

        // is there a firing servo?
        if (servoShoot != null) {
            // what should the position be?
            double posSh = (gamepad1.right_bumper)? posShooterFire : posShooterRest;

            // if that position has not been commanded, then command it
            if (posSh != posShooter) {
                posShooter = posSh;
                servoShoot.setPosition(posShooter);
            }
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
                // set 60 rps
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
            telemetry.addData("Shooter (from angle)", "vel %f", 20.0 * dcmotorShooter.getVelocity(AngleUnit.DEGREES)/360.0);
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
