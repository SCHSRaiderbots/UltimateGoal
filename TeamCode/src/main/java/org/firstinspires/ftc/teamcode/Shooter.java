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
        // TODO: investigate the type of motoer and its tick count
        // I believe it is listed as a VersaPlanetary, but no gear ratio is specifified
        // use RUN_TO_POSITION to figure out encoder counts
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
                // dcmotorShooter.setPower(0.5);

                // try controlling the velocity
                dcmotorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // trying to get 1 rps, but failing
                // spins way too fast.
                // try fudge factor of 28
                //   1 rps -- intermittent motion! (need to adjust PIDF?) (Battery was also low)
                //   10 rps -- better control, but will not shoot
                //   50 rps -- marginal shooter; reports 1.78/3.3
                //  100 rps -- maxed out velocity
                dcmotorShooter.setVelocity(360.0 * 50.0 / 28.0, AngleUnit.DEGREES);
                // dcmotorShooter.setVelocity(28);
            }

            if (!bShooter && gamepad1.x) {
                // remember the shooter is on
                bShooter = true;

                // turn on the motor
                dcmotorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                dcmotorShooter.setVelocity(60.0 * 28.0);
                dcmotorShooter.setPower(1.0);
                // reads 84.0 rps when I expected 60.
            }

            if (!bShooter && gamepad1.y) {
                // remember the shooter is on
                bShooter = true;

                // turn on the motor
                // we'll use setPower() to start, but PID velocity control would be better
                // 3.2 rps reported, but that is suspect
                dcmotorShooter.setPower(1.0);
            }

            // if the driver presses a button
            if (gamepad1.b) {
                // remember the shooter is off
                bShooter = false;

                // turn off the motor
                dcmotorShooter.setPower(0.0);

                // restore default run mode
                dcmotorShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // report the motor velocity
            // try revs per second
            telemetry.addData("Shooter", "vel %f", 28.0 * dcmotorShooter.getVelocity(AngleUnit.DEGREES)/360.0);
        }
        else {
            telemetry.addData("Shooter", "not present");
        }

    }

    @Override
    public void stop() {

    }
}
