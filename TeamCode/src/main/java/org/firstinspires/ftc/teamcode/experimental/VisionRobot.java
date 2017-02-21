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
    private State state = State.BUSY;
    private VisionOpMode opMode = null;

    private long lastStageTime = 0;

    private double beaconConfidence = 0; //TODO real value or set in init
    private double initialBeaconConfidence = 0; //TODO real value
    private int slidingConfidencePeriod = 1; //TODO real value

    /**
     * Current state or state of previous action
     */
    public enum State {
        BUSY, //Robot is in the middle of a task
        SUCCESS, //Last action was successful
        FAILURE_TECH, //Failure for technical reasons (i.e. beacon navigation lost sight of beacon)
        FAILURE_TIMEOUT, //Robot timed out on previous task
        CANCELLED //Previous action was cancelled
    }

    /**
     * Alliance enum for use in hitting techniques
     */
    public enum Alliance {
        RED, BLUE
    }

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
     *
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
     * Is the robot busy? (Descriptive commenting thanks to FIRST API)
     * @return Returns true if the robot is busy
     */
    public boolean isBusy() {
        return state == State.BUSY;
    }

    /**
     * Cancels any action currently going on (mostly for emergencies)
     */
    public void cancel() {
        //TODO
    }

    /**
     * navigates to beacon (until beacon goes out of camera view)
     *
     * @param Kp      proportional constant
     * @param Kd      differential constant
     * @param maxTime max time to go before quitting
     */
    public void PDtoBeacon(double Kp, double Kd, double maxTime) {
        //TODO
    }

    /**
     * Sets beacon confidence
     *
     * @param beaconConfidence Minimum confidence which must be maintained to continue navigation
     */
    public void setBeaconConfidence(double beaconConfidence) {
        this.beaconConfidence = beaconConfidence;
    }

    /**
     * Sets initial beacon confidence
     *
     * @param initialBeaconConfidence Minimum confidence which must be maintained to start navigation
     */
    public void setInitialBeaconConfidence(double initialBeaconConfidence) {
        this.initialBeaconConfidence = initialBeaconConfidence;
    }

    /**
     * Drives until beacon is found or failure due to timeout
     *
     * @param leftPower  Power of left side (with camera end as front)
     * @param rightPower Power of right side (with camera end as front)
     */
    public void detectBeacon(double leftPower, double rightPower) {
        //TODO
    }

    /**
     * Drive a certain amount of time with constant power
     *
     * @param leftPower  Left drive power (with camera end as front)
     * @param rightPower Right drive power (with camera end as front)
     * @param time
     */
    public void timeDrive(double leftPower, double rightPower, long time) {
        //TODO
    }

    /**
     * Gets beacon color (in nicely packaged object from lasarobotics)
     *
     * @return Returns BeaconColor object for raw use (will be used a lot later)
     */
    public Beacon.BeaconColor beaconColor() {
        //TODO (and decide if this is what we want)
        return null;
    }

    /**
     * Hits beacon from close up (should be called after PDtoBeacon)
     *
     * @param leftPower  Left motor power (with camera as front)
     * @param rightPower Right motor power (with camera as front)
     * @param maxTime    Max time to drive before failure due to timeout
     */
    public void hitBeacon(double leftPower, double rightPower, long maxTime) {
        //TODO
    }

    /**
     * Backs up from beacon to
     *
     * @param leftPower  Left motor power (with camera as front)
     * @param rightPower Right motor power (with camera as front)
     * @param time       Time to drive before stopping
     */
    public void backupFromBeacon(double leftPower, double rightPower, long time) {
        //TODO
    }

    /**
     * Determines if the beacon needs to be clicked
     *
     * @param alliance Alliance color
     */
    public void needToClick(Alliance alliance) {
        //TODO
        //TODO if beacon confidence is below threshold possibly tech failure but still return
    }

    /**
     * Logs a ton of data to telemetry (i.e. beacon location details)
     */
    public void logData() {
        //TODO
    }
}
