package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name="Vision Test", group="Auto Test")
public class VisionTest extends OpMode {
    // the assets directory has the .tflite for Ultimate Goal
    // it does not have a .tflite for last year's SkyStone game
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

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
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData("  left, top", "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData("  right, bottom", "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());

                    i = i + 1;
                }
                telemetry.update();
            }
        }
    }

    @Override
    public void start() {
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
        //
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

        // set the camera
        // TODO: choose phone camera or choose a webcam
        parameters.cameraDirection = CameraDirection.BACK;
        // parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // create a TFOD parameters object
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        // set the confidence parameter
        tfodParameters.minResultConfidence = 0.8f;

        // build a TensorFlow object detector
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // load the objects to detect from the assets directory
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
