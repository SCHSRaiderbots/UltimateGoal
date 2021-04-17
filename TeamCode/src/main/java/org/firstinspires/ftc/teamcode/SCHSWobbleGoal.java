package org.firstinspires.ftc.teamcode;

import android.util.Log;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

class SCHSWobbleGoal {

    private DcMotorEx motorWobble;
    private Servo wobbleServo;
    private boolean isDepositing = false;

    public void initialize(HardwareMap hardwareMap) {
        motorWobble = hardwareMap.get(DcMotorEx.class, "grabberMotor");
        wobbleServo = hardwareMap.get(Servo.class, "grabberServo");

        motorWobble.setDirection(DcMotorSimple.Direction.FORWARD);
        wobbleServo.setDirection(Servo.Direction.FORWARD);
        wobbleServo.setPosition(0);

        double F = 32767.0 / (2.0 * 288.0);
        PIDFCoefficients pidfRUE = new PIDFCoefficients(10.0, 1.0, 0.0, F, MotorControlAlgorithm.PIDF );
        PIDFCoefficients pidfR2P = new PIDFCoefficients(10.0, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);
        motorWobble.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUE);
        motorWobble.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfR2P);
        motorWobble.setDirection(DcMotorSimple.Direction.FORWARD);
        motorWobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorWobble.setTargetPosition(0);
        motorWobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorWobble.setPower(1.0);

    }

    public void depositWobble() {
        motorWobble.setTargetPosition(WOBBLE_ARM_DIST);
        motorWobble.setPower(0.8);
        while ((Math.abs(motorWobble.getCurrentPosition() - motorWobble.getTargetPosition()) > 10)) {
            Log.d("SCHSWobble: ", "Waiting for wobble arm to move");
        }
        sleep(1500);
        openGrabber();
    }

    public void liftWobble() {
        motorWobble.setTargetPosition(0);
        motorWobble.setPower(0.8);
    }

    public boolean isMoving() {
        if (motorWobble.getVelocity() == 0) {
            return false;
        }
        return true;
    }

    public void openGrabber() {
        wobbleServo.setPosition(0.4);
    }

}


