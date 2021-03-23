package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="SCHSController", group="SCHS")
@Disabled
public class SCHSController extends OpMode {

    @Override
    public void init() {
        //initialize objects for chassis, shooter, etc
        telemetry.addLine("Done Initializing");
    }

    @Override
    public void start() {
        //Set up robot devices, initial state, and game timer

        //runtime.reset();
        //newState(State.STATE_NAME);
    }

    @Override
    public void loop() {
        //state machine goes here
        // Send the current state info (state and time) back to first line of driver station telemetry.
    }

}
