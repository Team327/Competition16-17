package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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

    //stuff for find_beacon_1
    LinkedList<Double> slidingConfidence = null;
    static final double MIN_CONFIDENCE_MEAN = 0.50; //TODO determine value
    static final int CONFIDENCE_WINDOW_PERIOD = 5; //TODO determine optimum

    //stuff for move_beacon_1
    boolean startedToBeacon1 = false; //set to true as soon as it starts
    double prevError = 0; //previous error (for differential)
    double prevTime = 0; //previous time
    double Kp = 1; //Proportional constant //TODO find experimentally
    double Kd = -1; //Differntial constant (negative) //TODO find experimentally
    static final double FRAME_SIZE_BUFFER = 0; //Amount of space between frame height and beacon height before moving on //TODO find experimentally
    static final double MIN_CONFIDENCE = 0.40; //TODO determine experimentally

    //stuff for close_to_beacon_1
    boolean startedCloseToBeacon1 = false;
    double closeKp = 1; //Close proportional constant //TODO find experimentally
    double closeKd = -1; //Close differntial constant (negative) //TODO find experimentally
    double prevCloseError = 0;
    double prevCloseTime = 0;
    static final double CLOSE_DRIVE_POWER = 0.3;
    static final double PRESSABLE_DISTANCE = 0.2; //Optical sensor light/maxLight when ready to press //TODO find experimentally

    //stuff for pressing buttons
    int rightRed = 0; //Counts the number of hits for right red during goToBeacon_1
    int leftRed = 0; //Counts the number of hits for left red during goToBeacon_1
    static final double PRESSED_DISTANCE = 0.1; //Optical sensor light/maxLight when pressed //TODO find experimentally
    static final double PRESS_RATE = 0.01; //Change servo distance between loop iterations //TODO find experimentally

    public enum STATE {INIT,TO_BALL,BUMP_BALL,FIND_BEACON_1,MOVE_BEACON_1,CLOSE_TO_BEACON_1,HIT_BEACON_1,DONE}
    public STATE stage = STATE.INIT;

    //Time constants for autoadvance of autonomous mode
    public static final int TIME_BALL = 3000;
    public static final int TIME_TO_FIND_BEACON_1 = 5000;//millis
    public static final int TIME_TO_MOVE_BEACON_1 = 5000;
    public static final int TIME_TO_LEAVE_BEACON_1 = 5000;

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
                /*
                move servo fully left
                slowly move right until beacon identified
                center beacon on camera by moving servo and center servo on robot
                    robot should directly face the beacon
                 */
                findBeacon();
                break;

            case MOVE_BEACON_1:
                /*
                Leave servo centered on robot
                use the beacon offset from the center to navigate
                    if to the right, lower right power
                    if to the left, lower left power
                    else, full power
                when beacon leaves visible area, drive straight and hope :)

                Identify which side is red, for hitting beacon
                */
                goToBeacon();
                break;

            case CLOSE_TO_BEACON_1:
                //TODO implement
                /*
                When beacon goes out of camera range
                Use sensors on top to get aligned with beacon more and sense distance as driving forward
                Use same PD controller as MOVE_BEACON_1
                 */
                gotoLastStretch();
                break;

            case HIT_BEACON_1:
                //TODO implement
                /*
                Use identification of the beacon to hit the correct side
                 */
                clickBeacon();
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
            lastStageStart=System.currentTimeMillis(); //save the time of the change
        }
    }

    //TODO test
    void findBeacon()
    {
        if(lastStageStart - System.currentTimeMillis() > TIME_TO_FIND_BEACON_1) {
            //TODO time halt
        }
        double confidence = beacon.getAnalysis().getConfidence();
        slidingConfidence.add(confidence);
        if(slidingConfidence.size() < CONFIDENCE_WINDOW_PERIOD) {
            slidingConfidence.remove(0);
        }
        double meanConfidence = mean(slidingConfidence);
        if(meanConfidence > MIN_CONFIDENCE_MEAN) {
            left.setPower(0); //Stop motors because beacon is found
            right.setPower(0); // "
            stage = STATE.MOVE_BEACON_1;
            lastStageStart = System.currentTimeMillis();
        } else {
            left.setPower(1); //Set left to go backwards
            //TODO this may not need to go max power
        }

        //TODO test this
    }

    /**
     * Finds mean value of list
     * @param data list of doubles
     * @return mean value
     */
    double mean(List<Double> data) {
        double sum = 0;
        for(double value : data) {
            sum += value;
        }
        return sum / data.size();
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

    //TODO test
    void goToBeacon()
    {
        if(lastStageStart - System.currentTimeMillis() > TIME_TO_MOVE_BEACON_1) {
            //TODO time halt
        }

        Beacon.BeaconAnalysis anal = beacon.getAnalysis();
        Size frameSize = getFrameSize();
        if(anal.isBeaconFound() && anal.getConfidence() > MIN_CONFIDENCE) {
            double beaconHeight = anal.getHeight();
            double beaconWidth = anal.getWidth();
            double frameHeight = frameSize.height;
            double frameWidth = frameSize.width;
            if(frameHeight - beaconHeight >= FRAME_SIZE_BUFFER
                    || frameWidth - beaconWidth >= FRAME_SIZE_BUFFER) {
                stage = STATE.CLOSE_TO_BEACON_1;
                lastStageStart = System.currentTimeMillis();
            }
            //TODO use telemetry to ensure that frame size and beacon size are the same scale

            double beaconCenterX = anal.getCenter().x; //beacon center x
            double frameCenterX = frameSize.width / 2; //frame center x
            double error = frameCenterX - beaconCenterX; //error in x from beacon (right is positive)

            double time = System.nanoTime();

            if(startedToBeacon1) {
                double diff = (error - prevError) / (time - prevTime); //error differential
                double steering = Kp * error + Kd * diff; //PD steering uses error and diff times constants
                left.setPower(Range.clip(1 + (steering < 0 ? steering : 0), 0, 1)); //brake left if steering less than zero; clipped [0,1]
                right.setPower(Range.clip(-1 + (steering > 0 ? steering : 0), 0, 1)); //brake right if steering greater than zero; clipped [0,1]
                prevError = error;
                prevTime = time;
            } else {
                prevError = error; //If this is the first time, only get the error to prevError
                prevTime = time;
                startedToBeacon1 = true;
            }

            if(anal.isRightRed()) {
                rightRed++; //Add to right count if right is red
            }

            if(anal.isLeftRed()) {
                leftRed++; //Add to left count if left is red
            }
        } else {
            //TODO if beacon not found (this is temporary and is stopping not ideal)
            left.setPower(0);
            right.setPower(0);
        }
    }

    /*
    Does the last section of going to beacon 1 during STATE.CLOSE_TO_BEACON_1
     */
    //TODO test
    void gotoLastStretch() {
        double distance1 = dist1.getRawLightDetected() / dist1.getRawLightDetectedMax();
        double distance2 = dist2.getRawLightDetected() / dist2.getRawLightDetectedMax();
        if (distance1 < PRESSABLE_DISTANCE && distance2 < PRESSABLE_DISTANCE) {
            //Case the robot is close enough
            right.setPower(0);
            left.setPower(0);
            lastStageStart = System.currentTimeMillis();
            stage = STATE.HIT_BEACON_1;
        } else if (startedCloseToBeacon1) {
            //Case the robot needs to get closer and adjust
            double closeError = distance2 - distance2; //error = difference in distances
            double closeTime = System.nanoTime();
            double closeDiff = (closeError - prevCloseError) / (closeTime - prevCloseTime);
            double correction = closeKp * closeError + closeKd * closeDiff; //PD controller
            beacon_hitter.setPosition(Range.clip(beacon_hitter.getPosition() + correction, 0, 1));
            prevCloseError = closeError;
            prevCloseTime = closeTime;
        } else {
            //init close_to_beacon_1
            beacon_hitter.setPosition(0.5); //put servo in the middle
            prevCloseError = distance2 - distance2; //error = difference in distances; save to prev
            startedCloseToBeacon1 = true;
            right.setPower(CLOSE_DRIVE_POWER);
            left.setPower(-CLOSE_DRIVE_POWER);
            prevCloseTime = System.nanoTime();
        }
    }

    void clickBeacon()
    {
        double distance1 = dist1.getRawLightDetected() / dist1.getRawLightDetectedMax();
        double distance2 = dist2.getRawLightDetected() / dist2.getRawLightDetectedMax();
        if((leftRed > rightRed ? distance1 : distance2) <= PRESSED_DISTANCE) { //TODO make sure directions are right
            lastStageStart = System.currentTimeMillis();
            stage = STATE.DONE;
        } else {
            double change = leftRed > rightRed ? PRESS_RATE : -PRESS_RATE; //TODO make sure directions are right
            beacon_hitter.setPosition(Range.clip(beacon_hitter.getPosition() + change, 0, 1));
        }
    }
}
