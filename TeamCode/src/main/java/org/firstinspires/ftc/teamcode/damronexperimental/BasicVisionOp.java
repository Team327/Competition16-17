package org.firstinspires.ftc.teamcode.damronexperimental;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.ftc.resq.Constants;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by gssmrobotics on 11/18/2016.
 */

public class BasicVisionOp extends LinearVisionOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Wait for vision to initialize
        waitForVisionStart();

        // Camera (PRIMARY = standard rear camera)
        this.setCamera(Cameras.PRIMARY);

        // Set frame size
        this.setFrameSize(new Size(900, 900));

        // Enable extensions
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        // Analysis Method
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        // 0 is default, -1 is minimum and 1 is maximum tolerance
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        // Rotation data
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        //Camera control settings
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setManualExposureCompensation(Constants.EXPO_COMP);

        DcMotor front_left, front_right, back_left, back_right;
        Servo beaconHitter;
        OpticalDistanceSensor distLeft, distRight;

        // Wait for start of match
        waitForStart();

        front_left = hardwareMap.dcMotor.get("frontLeft");
        front_right = hardwareMap.dcMotor.get("frontRight");
        back_left = hardwareMap.dcMotor.get("backLeft");
        back_right = hardwareMap.dcMotor.get("backRight");
        beaconHitter = hardwareMap.servo.get("beaconHitter");
        distLeft = hardwareMap.opticalDistanceSensor.get("distLeft");
        distRight = hardwareMap.opticalDistanceSensor.get("distRight");

        //Initialize sensors and servo
        beaconHitter.setPosition(.5);
        distLeft.enableLed(true);
        distRight.enableLed(true);

        while (opModeIsActive()) {

            //Log a few things
            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Center", beacon.getAnalysis().getLocationString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("Beacon Buttons", beacon.getAnalysis().getButtonString());
            telemetry.addData("Screen Rotation", rotation.getScreenOrientationActual());
            telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
            telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);

            waitOneFullHardwareCycle();
        }
    }
}
