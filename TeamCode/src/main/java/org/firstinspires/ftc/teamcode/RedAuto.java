package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.ftc.resq.Constants;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

import java.util.LinkedList;
import java.util.List;

/**
 * Created by gssmrobotics on 11/21/2016.
 */
@Autonomous(name="Red Autonomoose")
public class RedAuto extends VisionOpMode {

    DcMotor right = null;
    DcMotor left = null;

    private double dist1Last,dist2Last;
    private OpticalDistanceSensor dist1 = null , dist2 = null;
    private Servo beacon_hitter = null;
    private double lastError=0;
    private long lastTime= System.nanoTime();

    private long lastStageStart;

    //stuff for vision
    LinkedList<Double> slidingConfidence = null;

    public enum STATE {INIT,TO_BALL,BUMP_BALL,FIND_BEACON_1,MOVE_BEACON_1,HIT_BEACON_1,LEAVE_BEACON_1,FIND_BEACON_2,MOVE_BEACON_2,HIT_BEACON_2,DONE}
    public STATE stage = STATE.INIT;

    //Time constants for autoadvance of autonomous mode
    public static final int TIME_BALL = 3000;
    public static final int TIME_TO_FIND_BEACON_1 = 5000;
    public static final int TIME_TO_MOVE_BEACON1 = 5000;
    public static final int TIME_TO_LEAVE_BEACON = 5000;
    public static final int TIME_TO_FIND_BEACON_2 = 1000;
    public static final int TIME_TO_MOVE_BEACON2 = 1000;

    @Override
    public void loop()
    {
        switch(stage)
        {
            case INIT: //Checks if the robot just started running
                initialize();
                //no break here to continue immediately to moving to ball
                
                
            /*
             We will drive directly at the ball until the distance sensors on the beacon hitter
             register the balls.
              */
            case TO_BALL:

                goToBall();
                break;

            /*
            We next us the beacon hitter to push the ball away from the direction we want to go.
            This means to the right on the red side.
             */
            case BUMP_BALL:
                pushBall(1000);
                break;

            case FIND_BEACON_1:
                //TODO Hunter identify beacon
                /*
                move servo fully left
                slowly move right until beacon identified
                center beacon on camera by moving servo and center servo on robot
                    robot should directly face the beacon
                 */
                break;

            case MOVE_BEACON_1:
                //TODO implement
                /*
                Leave servo centered on robot
                use the beacon offset from the center to navigate
                    if to the right, lower right power
                    if to the left, lower left power
                    else, full power
                when beacon leaves visible area, drive straight and hope :)

                Identify which side is red, for hitting beacon
                */
                break;

            case HIT_BEACON_1:
                //TODO implement
                /*
                Use identification of the beacon to hit the correct side
                 */
                break;

            case LEAVE_BEACON_1:
                //TODO Implement
                /*
                back up and to left for a time
                 */
                break;

            case FIND_BEACON_2:
                //TODO Hunter identify beacon
                /*
                move servo fully right
                slowly move left until beacon identified
                center beacon on camera by moving servo and center servo on robot
                    robot should directly face the beacon
                 */
                break;

            case MOVE_BEACON_2:
                //TODO implement
                /*
                Leave servo centered on robot
                use the beacon offset from the center to navigate
                    if to the right, lower right power
                    if to the left, lower left power
                    else, full power
                when beacon leaves visible area, drive straight and hope :)

                Identify which side is red, for hitting beacon
                */
                break;
            case HIT_BEACON_2:
                //TODO implement
                /*
                Use identification of the beacon to hit the correct side
                 */
                break;
            case DONE:
                stop();
        }

    }

    /*
    TODO check vision stuffs
     */
    void initialize()
    {
        ////Below are taken from visionsamples/BasicVisionSample.java
        //VisionOpMode initialization
        super.init();

        //select camera
        //  PRIMARY is non-selfie camera
        this.setCamera(Cameras.PRIMARY);

        //Set frame size
        this.setFrameSize(new Size(900, 900));

        //enable camera extensions
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        //Set analysis method
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        //set color tolerance
        //TODO play around with tolerance values
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        //camera control stuff
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        //camera control extension specifications
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setManualExposureCompensation(Constants.EXPO_COMP);
        ////thus ends the camera initialization

        //assign all motors
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");

        //Instantiate the sensor and servos
        beacon_hitter = hardwareMap.servo.get("beacon_hitter");
        dist1 = hardwareMap.opticalDistanceSensor.get("dist1");
        dist2 = hardwareMap.opticalDistanceSensor.get("dist2");

        //Initialize sensors and servo
        beacon_hitter.setPosition(.5);
        dist1.enableLed(true);
        dist2.enableLed(true);

        //Stuff for vision
        slidingConfidence = new LinkedList<Double>();

        //save the time of the start of to ball stage
        lastStageStart=System.currentTimeMillis();

        //set the current state to TO_BALL to begin moving toward it
        stage = STATE.TO_BALL;
    }

    /*
    TODO Test
     */
    void goToBall()
    {
        right.setPower(1);
        left.setPower(-1);

        //check if one of the distance sensors is close enough to the ball
        if(dist1.getRawLightDetected()/dist1.getRawLightDetectedMax()>.06 || dist1.getRawLightDetected()/dist1.getRawLightDetectedMax()>.06)
        {
            right.setPower(0);
            left.setPower(0);
            lastStageStart=System.currentTimeMillis(); //save the time of the change
            stage=STATE.BUMP_BALL; //advance a stage
        }
        if(System.currentTimeMillis()-lastStageStart>5000) //if time is over a threshold, move on to beacon1
        {
            right.setPower(0);

            left.setPower(0);
            lastStageStart=System.currentTimeMillis(); //save the time of the change
            stage=STATE.FIND_BEACON_1; //advance a stage
        }
    }

    /*
    TODO Test
     */
    void pushBall(long duration) {
        beacon_hitter.setPosition(.3);
        if(System.currentTimeMillis()-lastStageStart>1000)
        {
            stage=STATE.FIND_BEACON_1;
            lastStageStart=System.currentTimeMillis();
        }
    }

    void init_find_beacon()
    {
        beacon_hitter.setPosition(0); //Set beacon hitter to full left
    }

    void findBeacon_1()
    {
        Beacon.BeaconAnalysis anal = beacon.getAnalysis();
        double centerX = anal.getCenter().x;
        double confidence = anal.getConfidence();
    }

    /**
     * Finds the standard deviation of a list of doubles
     * @param data List of doubles
     * @return std deviation
     */
    double stdDeviation(List<Double> data)
    {
        if(data.size() == 0)
            return 0.0;
        double sum = 0;
        double sq_sum = 0;
        for(int i = 0; i < data.size(); ++i) {
            sum += data.get(i);
            sq_sum += data.get(i) * data.get(i);
        }
        double mean = sum / data.size();
        double variance = sq_sum / data.size() - mean * mean;
        return Math.sqrt(variance);
    }

    void goToBeacon()
    {

        //TODO do this
    }

    void alignBeaconPusher()
    {
        //TODO do this
    }

    void clickBeacon()
    {
        //TODO do this
    }
}
