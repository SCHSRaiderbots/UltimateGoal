package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Controller 2", group="Controller Test")
//@Disabled
public class ControllerTest extends LinearOpMode {

    //This OpMode will be used to begin testing all components together (with driver control) as they are built
    //Currently includes ability to drive robot chassis and shooter with 2 motors

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotorEx shooterMotor1 = null;
    //private DcMotorEx shooterMotor2 = null;

    private DcMotorEx intakeMotor = null;
    private Servo armGripper = null;
    private Servo shooterServo = null;
    private DcMotorEx grabberMotor = null;
    private DcMotorEx shooterMotor1 = null;
    private DcMotorEx shooterMotor2 = null;

    static final double MAX_POWER = 1.0;
    static final double POS_REST = 0.0;
    static final double POS_FIRE = 0.2;
    double BLAST_SPEED = 2000;
    double HALF_BLAST = 1000;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        //shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");

       intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
       grabberMotor = hardwareMap.get(DcMotorEx.class, "wobbleArm");
       shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
       shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");


        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        armGripper = hardwareMap.get(Servo.class, "armGripper");

        shooterServo.setPosition(POS_REST);

        // Needs one side to be reversed and one side to be forward so that ring shoots out in correct direction
        // Wheels are on either side of ring, so reverse one motor to run backwards to allow ring to launch forward
        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry










            //open and close grabber servo with x and y buttons on gamepad2
            if (gamepad2.x) {
                armGripper.setPosition(0);
            }

            else if (gamepad2.y) {
                armGripper.setPosition(0.4);
            }


            //turn wobble goal arm (grabberMotor) by moving left joystick up and down on gamepad2
            double grabberPower = -gamepad2.left_stick_y;
            grabberPower = Range.clip(grabberPower, -1.0, 1.0);

            int holdPosition = 0; //placeholder
            int timesChecked = 0;

            if (grabberPower == 0) {  //not optimal as it resets the mode every cycle
                if (timesChecked == 0) {
                    holdPosition = grabberMotor.getCurrentPosition();
                    timesChecked++;
                }
                grabberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabberMotor.setTargetPosition(holdPosition);
                grabberMotor.setPower(0.4); //random value because powerArm is 0
            } else {
                grabberMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (grabberPower < 0) { //faster up, slower down
                    grabberMotor.setPower(grabberPower/2.56);
                }
                if (grabberPower > 0)
                    grabberMotor.setPower(grabberPower/1.77);
                timesChecked = 0;
            }

            grabberMotor.setPower(grabberPower);


            if (gamepad2.dpad_up){
                intakeMotor.setPower(MAX_POWER);
            }

            if(gamepad2.a){
                shooterServo.setPosition(POS_REST);
            }

            else if(gamepad2.b){
                shooterServo.setPosition(POS_FIRE);
            }


            if (gamepad2.right_bumper) {
                shooterMotor1.setVelocity(BLAST_SPEED);
                shooterMotor2.setVelocity(BLAST_SPEED);
            }


            if (gamepad2.left_bumper) {
                shooterMotor1.setVelocity(HALF_BLAST);
                shooterMotor2.setVelocity(HALF_BLAST);
            }


            double speed = (int)(gamepad2.right_stick_y * 2000);
            speed = Range.clip(speed,-2000,2000);
            shooterMotor1.setVelocity(speed);
            shooterMotor2.setVelocity(speed);














            // Show the elapsed game time and wheel power and shooter motor velocity (input and actual)
            telemetry.addData("Status", "Run Time: " + runtime.toString());
        //    telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
          //  telemetry.addData("Status", "Motor velocity input (radians): " + velocity);
            //telemetry.addData("Status", "Motor velocity actual (radians): " + shooterMotor.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("Status", "Shooter Motor 1 velocity actual (tick/sec): " + shooterMotor1.getVelocity());
            telemetry.addData("Status", "Shooter Motor 2 velocity actual (tick/sec): " + shooterMotor2.getVelocity());
            telemetry.addData("Status", "Grabber Servo Position: " + armGripper.getPosition());
            telemetry.update();
        }
    }
}
