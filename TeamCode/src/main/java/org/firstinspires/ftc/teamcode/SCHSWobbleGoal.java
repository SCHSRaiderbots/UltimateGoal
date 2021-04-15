package org.firstinspires.ftc.teamcode;

<<<<<<< HEAD
import android.util.Log;

<<<<<<< HEAD
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
=======
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Thread.sleep;


public class SCHSWobbleGoal {

    private DcMotor armMotor; //motor that extends and retracts the arm

    private Servo armGripper; //servo that tightens or loosens the arm gripper

    private int armEncoderCount;

    private final double  GRIPPER_OPEN = 0.4;

    private final double GRIPPER_CLOSED = 0.0;


    void initialize(HardwareMap hardwareMap) {

        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        armGripper = hardwareMap.get(Servo.class, "armGripper");

        armGripper.setPosition(GRIPPER_CLOSED);



        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets encoder value to 0
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.4); //sets power of arm


    }

    public void extendArm(int encoderCounts){ //moves arm certain amount of encode counts
        armMotor.setTargetPosition(encoderCounts);

        }

        public void retractArm(){ //retracts arm
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }





    public void moveGripper(boolean choice) { //opens or closes gripper, depending on if parameter is true or false

        if (choice) {
            armGripper.setPosition(GRIPPER_OPEN);
        }

        else{
            armGripper.setPosition(GRIPPER_CLOSED);
        }
    }



    public double getLiftPos() {  //gets Encoder Count of Lift
        armEncoderCount = armMotor.getCurrentPosition();
        return armEncoderCount;
    }


}


>>>>>>> master
=======
class SCHSWobbleGoal {
}
>>>>>>> master
