package org.firstinspires.ftc.teamcode;

import android.util.Log;

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
        armMotor.setPower(0.7); //sets power of arm


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


