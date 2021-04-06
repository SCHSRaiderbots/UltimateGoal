package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static org.firstinspires.ftc.teamcode.Motion.setRobot;
import static org.firstinspires.ftc.teamcode.Motion.setRobot2019;

@Autonomous(name="Vision Test", group="Auto Test")
public class VisionTest extends OpMode {
    // drive motors
    private DcMotorEx dcmotorLeft = null;
    private DcMotorEx dcmotorRight = null;
    private boolean boolPOVDrive = true;

    // the assets directory has the .tflite for Ultimate Goal
    // it does not have a .tflite for last year's SkyStone game
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    // TODO: Anisha and https://isscroberto.com/2018/10/05/android-remove-idea-folder-from-git/

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AUnX7nP/////AAABmZjfOTd2skx4p/r+LBA29VQAFar5mbPnEfGtcl78mMIqK+EtsUOR33zwyiDCmj1oYMUx0P4eWZGi6EMhZgTM66/5llx5azKwGGxGmTJUGotbAekyZgxYR7SWDme6xMYGR68jZcR9rkvJxfB1ZKFytPXWeRpwzSAQJ0VACF/hdguUyfA6SSkF2dnc/iH76TkSV3hA4zz0v3wjHfQmmNBvrtgPklvfOTX2f+G5tBfBq75PEx52LaX+tOPTtBajR9MFwVT26kcqFz2GJCEBgjO3PX1St0xNJBqbbudKvZ+B/6xWuVhwHVqwOgy/RsuHLBFskh4n9Ec1xnuB9uCnQXrrliEtcR1TbnmIEYTX6FZtxF5H";

    /**
     * vuforia is our instance of the Vuforia localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * tfod is our instance of the TensorFlow Object Detection engine.
     */
    private TFObjectDetector tfod;


    @Override
    public void init() {
        // initialize the drive
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        dcmotorLeft = hardwareMap.get(DcMotorEx.class, "leftMotor");
        dcmotorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        dcmotorLeft.setDirection(DcMotor.Direction.REVERSE);
        dcmotorRight.setDirection(DcMotor.Direction.FORWARD);

        // After the drive motors are configured, inform the Motion class
        // odometry
        setRobot(dcmotorLeft, dcmotorRight);
        // use an old robot
        // TODO: use a phantom switch to determine the actual robot
        setRobot2019();

        // initialize the vision system

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /*
          Activate TensorFlow Object Detection before we wait for the start command.
          Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
        if (tfod == null) {
            // failed to build an object detector
            Log.d("TensorFlow", "Failed to build tfod object");
        }
        else {
            // activate object detection
            tfod.activate();

            // TODO: setClippingMargins() might be useful...
            // tfod.setClippingMargins();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

    }

    @Override
    public void init_loop() {
        // drive
        Motion.updateRobotPose();

        // during init, look for rings

        // do we have an object detector?
        if (tfod != null) {
            // yes, we have an object detector...

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            // we presume that this often returns null
            //   the usual case would be the recornitions are still computing
            //   updates may take a couple seconds
            //   we should be hitting this test every 20 to 50 ms.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            // are there updated recognitions?
             if (updatedRecognitions != null) {
                // yes!

                 // TODO: During Ultimate goal, we expect
                 // 1 recognition of a "Quad" at the start
                 // after randomization, that will change. The expectation is
                 // 0 recognitions               = 0 rings, Landing Zone A
                 // 1 recognition of a "Single"  = 1 ring,  Landing Zone B
                 // 1 recognition of a "Quad"    = 4 rings, Landing Zone C
                 //
                 // TODO: Use vision to confirm setup
                 // The robot must start on a Start Line
                 // The ring stack is half-way between the start lines
                 // by looking at the initial quad stack, the robot
                 // could check whether it is on the left or the right start line
                 // Blue Alliance starts on the left side of the field
                 // Red Alliance starts on the right side of the field

                // report the number of objects
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                Log.d("# Object Detected", "size = " + updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData("tfod", recognition.getLabel());
                    telemetry.addData("confidence", recognition.getConfidence());
                    telemetry.addData("  left, top", "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData("  right, bottom", "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
                // telemetry.update();
            }
        }
    }

    @Override
    public void start() {
        // drive
        Motion.updateRobotPose();

        // assume we are done with vision

        // I may never hit Play. Turning off at stop() should always work.
        // lets just deactivate at start()
        if (tfod != null) {
            // this keeps the camera image display, but stops detection
            tfod.deactivate();
        }

    }

    @Override
    public void loop() {
        // drive
        Motion.updateRobotPose();

        double powerLeft;
        double powerRight;

        // Choose to drive using either Tank Mode, or POV Mode
        if (boolPOVDrive) {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            powerLeft = Range.clip(drive + turn, -1.0, 1.0);
            powerRight = Range.clip(drive - turn, -1.0, 1.0);
        }
        else {
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            powerLeft  = -gamepad1.left_stick_y ;
            powerRight = -gamepad1.right_stick_y ;
        }

        // Send calculated power to wheels
        dcmotorLeft.setPower(powerLeft);
        dcmotorRight.setPower(powerRight);
    }

    @Override
    public void stop() {
        //
        if (tfod != null) {
            // this closes everything down
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // set the license key
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // set the camera: either phone camera or webcam
        // .camera takes precedence
        // .cameraDirection
        // .cameraName
        if (false) {
            // I get a camera stream from the phone camera.
            // phone camera - I'm guessing it is the default
            parameters.cameraDirection = CameraDirection.BACK;
        } else {
            // I get a camera stream from the webcam.
            // TODO: The webcam is not reliable and keeps aborting transfers...
            // to configure a camera, go to configuration menu, select the configuration, and then click SCAN
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
            // should not need to set a direction
            // TODO: how to set resolution?
            // I have no idea which resolution the camera has selected
            // TODO: set calibration
            // downloaded 3D Zephyr Free.
            // see also https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
            // see also https://en.wikipedia.org/wiki/Distortion_(optics)
            // TODO: choose the filename
            // there were zero files
            Log.d("Webcam 1", String.valueOf(parameters.webcamCalibrationFiles.length));
            // look with                                       "res/xml/teamwebcamcalibrations.xml"
            // was /storage/emulated/0/FIRST/webcamcalibrations/res/xml/teamwebcamcalibrations.xml (No such file or directory)
            // shortened to just teamwebcamcalibrations.xml, but that file was not found.
            // from http://192.168.49.1:8080/?page=manage.html&pop=true one can upload a calibration file
            // ~/StudioProjects/UltimateGoal/TeamCode/src/main/res/xml/teamwebcamcalibrations.xml
            // uploading that file got me past the file not found
            parameters.addWebcamCalibrationFile("teamwebcamcalibrations.xml");
            // on open, fdDescriptors not available
            // not UVC!
            // vid 13028 pid 37424 serial=null product=HD USB Camera
            // isModeSupported(Fixed); true
            // vid=0x32e4,pid=0x9230 1920x1080 null
            //    640x480
            //    320x240
            //    1280x720
            //    1024x768
            //    800x600
            //    1280x1024
            // looks like 640x480 was chosen with f=0.000,0.000
            // for the ov2710, a quad stack is seen as a single.
        }

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // TODO: Trackables ...
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        // TODO: what implications for webcam?
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // create a TFOD parameters object
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        // set the confidence parameter
        // TODO: try lower confidence for tfod?
        tfodParameters.minResultConfidence = 0.8f;

        // build a TensorFlow object detector
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // load the objects to detect from the assets directory
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
