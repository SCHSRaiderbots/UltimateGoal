package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

@Autonomous(name="SCHSController", group="SCHS")
//@Disabled
public class SCHSController extends OpMode {

    private SCHSDrive rileyChassis = null;
    private SCHSDetection rileyEnv = null;
    private SCHSShooter rileyShooter = null;
    private SCHSWobbleGoal rileyWobble = null;
    private boolean isInitialized = false;
    private double leftDist;
    private double rightDist;
    private boolean isArcTurn;
    private int numRings;

    private enum State {
        STATE_INITIAL,
        STATE_DEPOSIT_WOBBLE,
        STATE_SHOOT_RINGS,
        STATE_MOVE_TO_SHOOT,
        STATE_GO_TO_TARGET,
        STATE_LOWER_WOBBLE_ARM,
        STATE_GO_TO_LAUNCH,

        STATE_TEST_1,
        STATE_TEST_2,

        STATE_STOP
    }

    private State currState; //current state machine
    private SCHSPathSeg[] currPath; // array holding current path
    private int currSeg; //index of leg of current path

    // time into round and time into current state
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime currStateTime = new ElapsedTime();

    @Override
    public void init() {
        //initialize objects for chassis, shooter, etc
        rileyChassis = new SCHSDrive();
        rileyChassis.initialize(hardwareMap);

        rileyShooter = new SCHSShooter();
        rileyShooter.initialize(hardwareMap);

        rileyWobble = new SCHSWobbleGoal();
        rileyWobble.initialize(hardwareMap);

        //rileyEnv = new SCHSDetection();
        //rileyEnv.initialize(hardwareMap);

        //moved resetting encoders to init
        rileyChassis.resetEncoders();

        telemetry.addLine("Done Initializing");

        msStuckDetectLoop = 20000;
        msStuckDetectInit = 20000;
        msStuckDetectInitLoop = 20000;

        /*
        //detect stack of rings here
        int initialRingNum = rileyEnv.detectNumRings();
        telemetry.addLine("SCHS: init num rings detected: " + initialRingNum);
        Log.d("SCHS:", "init num rings detected: " + initialRingNum);*/

    }

    @Override
    public void start() {
        //Set up robot devices, initial state, and game timer
        rileyChassis.setDrivePower(0,0);
        rileyChassis.setPoseInches(-36, -63, 90);

        runtime.reset();
        newState(State.STATE_INITIAL);
        //newState(State.STATE_STONES_FIND_DIST);
    }

    @Override
    public void loop() {
        rileyChassis.loop();
        // Send the current state info (state and time) back to first line of driver station telemetry.

        switch (currState) {
            case STATE_INITIAL:
                telemetry.addLine("SCHS: inside STATE_INITIAL");
                Log.d("SCHS:", "inside STATE_INITIAL");
                if (rileyChassis.encodersAtZero()){
                    //scan rings
                    /*numRings = rileyEnv.detectNumRings();

                telemetry.addLine("numRings:" + numRings);
                Log.d("SCHS: SCAN_RINGS", "numRings:" + numRings);*/

                    numRings = 0;
                    newState(State.STATE_MOVE_TO_SHOOT);
                    //newState((State.STATE_SCAN_RINGS));
                } else {
                    telemetry.addLine("SCHS: STATE_INITIAL else");
                    Log.d("SCHS:", "STATE_INITIAL else");
                }
                break;

            case STATE_MOVE_TO_SHOOT:
                telemetry.addLine("SCHS: inside STATE_MOVE_TO_SHOOT");
                Log.d("SCHS:", "inside STATE_MOVE_TO_SHOOT");
                if (rileyChassis.encodersAtZero()){
                    startPath(moveToShoot);
                    newState(State.STATE_SHOOT_RINGS);
                } else {
                    telemetry.addLine("SCHS: inside STATE_MOVE_TO_SHOOT");
                    Log.d("SCHS:", "inside STATE_MOVE_TO_SHOOT");
                }
                break;

            case STATE_SHOOT_RINGS:
                if (pathComplete(DRIVE, false)) {
                    telemetry.addLine("SCHS: inside STATE_SHOOT_RINGS");
                    Log.d("SCHS:", " inside STATE_SHOOT_RINGS");
                    //shoot 3 rings into high goal
                    rileyShooter.shoot(SHOOT_VEL);
                    newState(State.STATE_GO_TO_TARGET);
                } else {
                    telemetry.addLine("SCHS: inside STATE_SHOOT_RINGS else");
                    Log.d("SCHS:", " inside STATE_SHOOT_RINGS else");
                }
                break;

            case STATE_GO_TO_TARGET:
                if (!rileyShooter.getIsShooting()) {
                    telemetry.addLine("SCHS: inside STATE_GO_TO_TARGET");
                    Log.d("SCHS:", " inside STATE_GO_TO_TARGET");

                    if (numRings == 1) {
                        startPath(goToTargetB);
                    } else if (numRings == 4) {
                        startPath(goToTargetC);
                    } else {
                        startPath(goToTargetA);
                    }
                    newState(State.STATE_DEPOSIT_WOBBLE);
                } else {
                    telemetry.addLine("SCHS: inside STATE_GO_TO_TARGET else");
                    Log.d("SCHS:", " inside STATE_GO_TO_TARGET else");
                }
                break;

            case STATE_DEPOSIT_WOBBLE:
                if (pathComplete(DRIVE, false)) {
                    telemetry.addLine("SCHS: inside STATE_LOWER_WOBBLE_ARM");
                    Log.d("SCHS:", " inside STATE_LOWER_WOBBLE_ARM");
                    //lower wobble arm

                    rileyWobble.depositWobble();
                    newState(State.STATE_GO_TO_LAUNCH);
                } else {
                    telemetry.addLine("SCHS: inside STATE_LOWER_WOBBLE_ARM else");
                    Log.d("SCHS:", " inside STATE_LOWER_WOBBLE_ARM else");
                }
                break;

            case STATE_GO_TO_LAUNCH:
                telemetry.addLine("SCHS: inside STATE_GO_TO_LAUNCH");
                Log.d("SCHS:", " inside STATE_GO_TO_LAUNCH");

                if (!rileyWobble.isMoving()) {
                    if (numRings == 1) {
                        startPath(goToLaunchB);
                    } else if (numRings == 4) {
                        startPath(goToLaunchC);
                    } else {
                        startPath(goToLaunchA);
                    }
                    newState(State.STATE_STOP);
                } else {
                    telemetry.addLine("SCHS: inside STATE_GO_TO_LAUNCH else");
                    Log.d("SCHS:", " inside STATE_GO_TO_LAUNCH else");
                }
                break;

            case STATE_TEST_1:
                if (rileyChassis.encodersAtZero()) {
                    startPath(testPathRun);
                    newState(State.STATE_STOP);
                } else {
                    telemetry.addLine("SCHS: STATE_TEST_1 else");
                    Log.d("SCHS:", "STATE_TEST_1 else");
                }
                break;

            case STATE_STOP:
                if (pathComplete(DRIVE, false)) {
                    Log.d("SCHS", "inside STATES_STOP");
                    telemetry.addLine("STATES_STOP");
                } else {
                    telemetry.addLine("SCHS: STATE_STOP else");
                    Log.d("SCHS:", "STATE_STOP else");
                }
                break;
                
        }
    }

    @Override
    public void stop(){
        rileyChassis.useConstantSpeed();
        rileyChassis.setDrivePower(0,0);
    }

    public void newState(State newState) {
        //reset state time, change to next state
        currStateTime.reset();
        currState = newState;
    }

    public void startPath(SCHSPathSeg[] path){
        currPath = path;    // Initialize path array
        currSeg = 0;
        rileyChassis.synchEncoders(); // Lock in the current position
        startSeg();             // Execute the current (first) Leg
        //rileyChassis.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION); // Enable RunToPosition mode
    }

    public void startSeg() {
        int Left = 0;
        int Right = 0;

        if (currPath != null) {
            if (currPath[currSeg].isTwoSpeed == false && currPath[currSeg].armPart == 0) { //moving straight and turn in place
                // Load up the next motion based on the current segemnt.
                Left = (int) (currPath[currSeg].leftDist * COUNTS_PER_INCH);
                Right = (int) (currPath[currSeg].rightDist * COUNTS_PER_INCH);

                leftDist = Left;
                rightDist = Right;
                isArcTurn = false;

                rileyChassis.addEncoderTarget(Left, Right);
                rileyChassis.setDrivePower(currPath[currSeg].moveSpeed, currPath[currSeg].moveSpeed);
                rileyChassis.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION); // Enable RunToPosition mode
                telemetry.addLine("SCHS: startseg(): move straight/turn");
                Log.d("SCHS", "startseg(), target encoder left" + Left);
                Log.d("SCHS", "startseg(), target encoder right" + Right);
                Log.d("SCHS: startseg():", "move straight/turn");
            } else {//arc turn
                Left = (int) (currPath[currSeg].leftDist * COUNTS_PER_INCH);
                Right = (int) (currPath[currSeg].rightDist * COUNTS_PER_INCH);
                rileyChassis.addEncoderTarget(Left, Right);
                rileyChassis.setDrivePower(currPath[currSeg].leftSpeed, currPath[currSeg].rightSpeed);
                rileyChassis.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION); // Enable RunToPosition mode
                isArcTurn = true;
                telemetry.addLine("SCHS: startseg(): arc turn");
                Log.d("SCHS: startseg():", "arc turn");
            }
        }
        currSeg++;  // Move index to next segment of path
    }

    //checks if the current path is complete
    //As each segment completes, the next segment is started unless there are no more.
    //Returns true if the last leg has completed and the robot is stopped.

    private boolean pathComplete(int roboPart, boolean isBothShootDrive) {
        // Wait for this Segement to end and then see what's next.
        if (moveComplete(roboPart, isBothShootDrive)) {
            Log.d("SCHS: moveComplete", "moveComplete() true");

            // Start next Segement if there is one.
            if (currSeg < currPath.length)
            {
                Log.d("SCHS", "inside pathComplete() if, moveComplete = true");
                //telemetry.addLine("SCHS: pathComplete(): move arm extend before startseg" + rileyArm.getExtendPos());
                //Log.d("SCHS: pathComplete(): ", "move arm extend before startseg" + rileyArm.getExtendPos());
                startSeg();
                //telemetry.addLine("SCHS: pathComplete(): move arm extend after startseg" + rileyArm.getExtendPos());
                //Log.d("SCHS: pathComplete(): ", "move arm extend after startseg" + rileyArm.getExtendPos());

            }
            else  // Otherwise, stop and return done
            {
                Log.d("SCHS", "inside pathComplete() else, moveComplete = true");

                currPath= null;
                currSeg= 0;
                rileyChassis.setDrivePower(0, 0);
                rileyChassis.useConstantSpeed();

                //rileyArm.setArmPower(0, LIFT);
                //rileyArm.setArmPower(0, ARM);
                //rileyArm.useConstantSpeed(LIFT);
                //rileyArm.useConstantSpeed(ARM);

                return true;
            }
        }
        Log.d("SCHS: moveComplete", "moveComplete() false");
        //telemetry.addLine("SCHS: startseg(): move arm extend false" + rileyArm.getExtendPos());
        return false;
    }

    // Return true if motors have both reached the desired encoder target
    public boolean moveComplete(int roboPart, boolean isBothShootDrive) {
        if (roboPart == DRIVE ) {
                Log.d("SCHS: moveComplete", "inside moveComplete() for DRIVE");
                Log.d("SCHS", "moveComplete(), curr target left" + rileyChassis.getLeftEncoderTarget());
                Log.d("SCHS", "moveComplete(), curr target right" + rileyChassis.getRightEncoderTarget());
                Log.d("SCHS", "moveComplete(), curr encoder left" + rileyChassis.getLeftPosition());
                Log.d("SCHS", "moveComplete(), curr encoder right" + rileyChassis.getRightPosition());
                Log.d("SCHS", "finished moveComplete():" + ((Math.abs(rileyChassis.getLeftPosition() - rileyChassis.getLeftEncoderTarget()) < 20) &&
                        (Math.abs(rileyChassis.getRightPosition() - rileyChassis.getRightEncoderTarget()) < 20)));

                if ((leftDist > 3300 || rightDist > 3300) && !isArcTurn) {
                    return ((Math.abs(rileyChassis.getLeftPosition() - rileyChassis.getLeftEncoderTarget()) < 60) &&
                            (Math.abs(rileyChassis.getRightPosition() - rileyChassis.getRightEncoderTarget()) < 60));
                } else if (((Math.abs(leftDist) < 1870) && (Math.abs(leftDist) > 1855)) || ((Math.abs(rightDist) < 1870) && (Math.abs(rightDist) > 1855))) {
                    return ((Math.abs(rileyChassis.getLeftPosition() - rileyChassis.getLeftEncoderTarget()) < 60) &&
                            (Math.abs(rileyChassis.getRightPosition() - rileyChassis.getRightEncoderTarget()) < 60));
                } else {
                    return ((Math.abs(rileyChassis.getLeftPosition() - rileyChassis.getLeftEncoderTarget()) < 20) &&
                            (Math.abs(rileyChassis.getRightPosition() - rileyChassis.getRightEncoderTarget()) < 20)); //10, change to 25
                }
            } /*else if (roboPart == LIFT) {
                Log.d("SCHS:", "inside moveComplete() for LIFT");
                return (Math.abs(rileyArm.getLiftPos() - rileyArm.getLiftEncoderTarget()) < 25); //10, change to 25
            } else if (roboPart == ARM) {
                Log.d("SCHS:", "inside moveComplete() for ARM");
                Log.d("SCHS:", "rileyArm getExtendPos = " +rileyArm.getExtendPos());
                Log.d("SCHS:", "rileyArm getArmEncoderTarget = " +rileyArm.getArmEncoderTarget());
                return (Math.abs(rileyArm.getExtendPos() - rileyArm.getArmEncoderTarget()) < 10);
            } else if (isBothArmDrive) {
                Log.d("SCHS:", "inside moveComplete() arm + drive");
                if(roboPart == ARM || roboPart == DRIVE) {
                    Log.d("SCHS:", "executing arm+drive check moveComplete()");
                    boolean armDone = Math.abs(rileyArm.getExtendPos() - rileyArm.getArmEncoderTarget()) < 25; //<10 change to <25
                    boolean moveDone = (Math.abs(rileyChassis.getLeftPosition() - rileyChassis.getLeftEncoderTarget()) < 20) &&
                            (Math.abs(rileyChassis.getRightPosition() - rileyChassis.getRightEncoderTarget()) < 20);
                    return (armDone && moveDone);
                }
            }*/
            /* } else if(roboPart == LONG_DRIVE) {
                Log.d("SCHS:", "inside moveComplete() for LONG_DRIVE");
                //return rileyChassis.isMoveDone;
                return ((Math.abs(rileyChassis.getLeftPosition() - rileyChassis.getLeftEncoderTarget()) < 20) &&
                        (Math.abs(rileyChassis.getRightPosition() - rileyChassis.getRightEncoderTarget()) < 20)); //10, change to 25
            }*/

        Log.d("SCHS:", "moveComplete() no case entered");
        return false;
    }

}
