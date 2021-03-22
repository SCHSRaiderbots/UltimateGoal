package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Wobble", group="Iterative Opmode")
public class Wobble extends OpMode {

    // A core hex motor to move the arm
    private DcMotorEx motorWobble;

    // A servo to close the gripper
    private Servo servoGripper;
    private double posGripper;
    static final double GRIPPER_OPEN = 0.4;
    static final double GRIPPER_CLOSED = 0.0;

    @Override
    public void init() {
        // find the Wobble actuators

        // find the wobble motor for the arm
        motorWobble = hardwareMap.get(DcMotorEx.class, "motorWobble");
        // configure the motor
        PIDFCoefficients pidf = new PIDFCoefficients();
        pidf.p = 10;
        pidf.i = 0.0;
        pidf.d = 0.0;
        pidf.f = 0.0;
        pidf.algorithm = MotorControlAlgorithm.PIDF;
        // OK, I'm confused.
        // Default PIDF(rue) = 10, 3, 0, 0, LegacyPID
        //         PIDF(r2p) = ???
        // If I set
        //    PIDF(rue) = 10, 0, 0, 0, PIDF
        //    PIDF(r2p) = 10, 0, 0, 0, PIDF
        // then there is not enough power sent to the motor to lift it.
        // However
        //    PIDF(rue) = 10, 3, 0, 0, LegacyPID (the default)
        //    PIDF(r2p) = 10, 0, 0, 0, PIDF
        // does have the power to lift. That suggests that R2P DOES USE the RUE coefficients.
        // The lift happens because the integrated error supplies enough power to move the arm.
        // Does that mean that RUE and R2P are not what I expect them to be?
        pidf.i = 3.0;
        motorWobble.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        pidf.i = 0.0;
        motorWobble.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidf);
        motorWobble.setDirection(DcMotorSimple.Direction.REVERSE);
        // use the motor as a servo
        // then change the mode
        motorWobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // first set the target position
        motorWobble.setTargetPosition(0);
        // then set r2p mode
        motorWobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // try RUE -- No, this just runs the motor continuously -- even when RUE.algorithm is PIDF
        // motorWobble.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set the power (otherwise the motor will not move)
        motorWobble.setPower(1.0);

        // TODO: the PID values for the arm are horrible...
        LogDevice.dump("motorWobble", motorWobble);

        // find the servo gripper
        servoGripper = hardwareMap.get(Servo.class, "servoGripper");
        // command it open
        posGripper = GRIPPER_OPEN;
        servoGripper.setPosition(posGripper);

     }

    @Override
    public void init_loop() {
        // nothing to do
    }

    @Override
    public void start() {
        // nothing to do
    }

    @Override
    public void loop() {
        // use the right trigger to move the arm 1/4 turn
        int ticksTarget = (int)(gamepad1.right_trigger * 288);
        motorWobble.setTargetPosition(ticksTarget);
        telemetry.addData("Wobble", "target position = %d", ticksTarget);
        telemetry.addData("Wobble", "actual position = %d", motorWobble.getCurrentPosition());

        double posGripperCmd = (gamepad1.left_bumper)? GRIPPER_CLOSED : GRIPPER_OPEN;
        if (posGripperCmd != posGripper) {
            posGripper = posGripperCmd;
            servoGripper.setPosition(posGripper);
        }
    }

    @Override
    public void stop() {

    }
}
