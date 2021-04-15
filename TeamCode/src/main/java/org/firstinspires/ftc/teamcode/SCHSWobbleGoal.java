package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

        //set target position to current position
        motorWobble.setTargetPosition(motorWobble.getCurrentPosition());
        motorWobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set power level to make it move
        motorWobble.setPower(1.0);
    }

}
