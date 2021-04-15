package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

class SCHSShooter {

    private DcMotorEx shooterMotorLeft;
    private DcMotorEx shooterMotorRight;
    private Servo shooterServo;
    private boolean isShooting = false;

    public void initialize(HardwareMap hardwareMap) {
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        shooterMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.FORWARD);

        shooterMotorLeft.setVelocity(0);
        shooterMotorRight.setVelocity(0);
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //shoot 3 rings at start of auto
    public void shoot(double velocity) {
        isShooting = true;
        startShooters(velocity);
        sleep(3500);
        servoPushRing();
        sleep(1500);
        reloadServo();
        sleep(1500);
        servoPushRing();
        sleep(1500);
        reloadServo();
        sleep(1500);
        servoPushRing();
        sleep(1500);
        reloadServo();
        stopShooters();
        isShooting = false;
    }

    //check if robot is still shooting
    public boolean getIsShooting() {
        return isShooting;
    }

    //check is shooter is not moving
    public boolean isShooterStopped() {
        if (shooterMotorLeft.getVelocity() == 0 && shooterMotorRight.getVelocity() == 0 ) {
            return true;
        }
        return false;
    }

    public void startShooters(double velocity) {
        shooterMotorLeft.setVelocity(SHOOT_VEL);
        shooterMotorRight.setVelocity(SHOOT_VEL - 100);
    }

    public void stopShooters() {
        shooterMotorLeft.setVelocity(0);
        shooterMotorRight.setVelocity(0);
    }

    public void servoPushRing() {
        shooterServo.setPosition(0.2);
    }

    public void reloadServo() {
        shooterServo.setPosition(0);
    }
}
