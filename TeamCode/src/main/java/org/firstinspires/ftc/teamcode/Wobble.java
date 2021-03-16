package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Wobble", group="Iterative Opmode")
public class Wobble extends OpMode {

    // A core hex motor to move the arm
    private DcMotorEx motorWobble;

    // A servo to close the gripper
    private Servo servoGripper;

    @Override
    public void init() {
        // find the Wobble actuators

        // find the wobble motor for the arm
        motorWobble = hardwareMap.get(DcMotorEx.class, "motorWobble");
        // configure the motor
        motorWobble.setDirection(DcMotorSimple.Direction.FORWARD);
        // use the motor as a servo
        // first set the target position
        motorWobble.setTargetPosition(0);
        // then change the mode
        motorWobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set the power (otherwise the motor will not move)
        motorWobble.setPower(1.0);

        // TODO: the PID values for the arm are horrible...
        LogDevice.dump("motorWobble", motorWobble);

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
        // use the right trigger to move the arm 1/2 turn
        int ticksTarget = (int)(gamepad1.right_trigger * 288 * 0.5);
        motorWobble.setTargetPosition(ticksTarget);
        telemetry.addData("Wobble", "target position = %d", ticksTarget);
        telemetry.addData("Wobble", "actual position = %d", motorWobble.getCurrentPosition());
    }

    @Override
    public void stop() {

    }
}
