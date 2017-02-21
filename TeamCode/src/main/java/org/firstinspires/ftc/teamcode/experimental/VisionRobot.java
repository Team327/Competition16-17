package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.ftc.resq.Constants;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by gssmrobotics on 2/20/2017.
 */

public class VisionRobot extends Robot {
    private boolean busy = false; //True when in the middle of an action
    private VisionOpMode opMode = null;

    private long lastStageTime = 0;

    /**
     * Constructor for VisionRobot - extension of Robot class with vision
     *
     * @param map    HardwareMap for getting motor control
     * @param opMode VisionOpMode for vision stuff (probably just pass 'self')
     */
    public VisionRobot(HardwareMap map, VisionOpMode opMode) {
        super(map);
    }

    /**
     * Initialize vision stuff (Except enableExtension() calls)
     * <p>
     * The following must be in opmode initialization before this call:
     * """
     * enableExtension(Extensions.BEACON);         //Beacon detection
     * enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
     * enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control
     * """
     */
    private void init() {
        //Set primary camera to main camera on back
        opMode.setCamera(Cameras.PRIMARY);

        //Set frame size
        opMode.setFrameSize(new Size(900, 900));

        opMode.beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        //set color tolerance
        //TODO play around with tolerance values
        opMode.beacon.setColorToleranceRed(0);
        opMode.beacon.setColorToleranceBlue(0);

        //camera control stuff
        opMode.rotation.setIsUsingSecondaryCamera(false);
        opMode.rotation.disableAutoRotate();
        opMode.rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        //camera control extension specifications
        opMode.cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        opMode.cameraControl.setManualExposureCompensation(Constants.EXPO_COMP);
        ////thus ends the camera initialization
    }

    /**
     * Is the robot busy? (Desccriptive commenting thanks to FIRST API)
     * <p>
     * (It actually returns true if the robot is in the middle of a multi-step process)
     *
     * @return Returns true if the robot is busy
     */
    public boolean isBusy() {
        return busy;
    }

    /**
     * Cancels any action currently going on (mostly for emergencies)
     */
    public void cancel() {
        //TODO
    }


}
