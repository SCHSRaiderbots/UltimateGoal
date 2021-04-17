package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

public class SCHSDetection {
    protected int ringNum;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     *
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private VuforiaLocalizer.Parameters parameters;

    public SCHSDetection() {
        ringNum = 0;
    }

    public void initialize(HardwareMap hardwareMap) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia(hardwareMap);
        initTfod(hardwareMap);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.5, 1.78);
        }
    }

    public int detectNumRings(){
        long startTime = System.currentTimeMillis();
        int numRings = 0;
        boolean isDone = false;

        while((System.currentTimeMillis() - startTime)<= SCAN_TIME) {

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                Log.d("Status", "SCHSObjectDetection: # Object Detected " + updatedRecognitions.size());
                Log.d("Status", "SCHSObjectDetection: TIME: " + (System.currentTimeMillis()-startTime));

                if (updatedRecognitions.size() == 0) {
                    Log.d("SCHSDetection:", "0 rings in stack");
                    numRings = 0;
                    isDone = true;
                } else if (updatedRecognitions.size() == 1) {
                    if (updatedRecognitions.get(0).getLabel().equals(LABEL_FIRST_ELEMENT)) {
                        Log.d("SCHSDetection:", "4 rings in stack");
                        numRings = 4;
                        isDone = true;
                    } else if (updatedRecognitions.get(0).getLabel().equals(LABEL_SECOND_ELEMENT)) {
                        Log.d("SCHSDetection:", "1 rings in stack");
                        numRings = 1;
                        isDone = true;
                    } // end of if, quad vs single stack
                } else {
                    Log.d("SCHSDetection:", "UH OH! More than 1 object detected");
                    numRings = 0;
                    isDone = false;
                }
            }//end of if
            if (isDone){
                break;
            }
        }//end of while
        return numRings;
    }//end of detectNumRings()

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
