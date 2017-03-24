package org.firstinspires.ftc.teamcode.experimental;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.LinkedList;
import java.util.List;

import org.firstinspires.ftc.teamcode.experimental.States.*;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.experimental.States.isBusy;

/**
 * Created by gssmrobotics on 1/17/2017.
 */



public class Robot {
    protected DcMotor leftMotor, rightMotor, shooter, caroline, intake;
    protected Boolean forward = true;
    protected ModernRoboticsI2cRangeSensor frontDist, leftFrontDist, leftRearDist;
    protected I2cDeviceReader frontDistReader, leftFrontDistReader, leftRearDistReader;
    protected Servo beaconPusher, shooterBlock;     //capHolder

    public State state;
    private final State[] busyStates = {State.PD_BEACON, State.TIME_DRIVE, State.DIST_DRIVE,
            State.DRIVE2DIST, State.DETECT_BEACON, State.HIT_BEACON, State.BACKUP}; //TODO add more

    private long lastStageTime=0;

    //wall follow cached variables
    private double prevNanoTime = 0; //not actually in nanos but uses nanos for calculation
    private LinkedList<Double> errors; //sliding window of errors
    private final int errorsPeriod = 10; //period of sliding window for error (used in I)
    private double leftDistSeparation = 30; //TODO measure distance between sensors on bot

    private int leftLastPos = 0;
    private int rightLastPos = 0;
    private int shootLastPos = 0;

    //Shooting variables
    private ShootState shootState = ShootState.STOPPED;
    private long shootPrevTime = 0;
    private double prevTime = 0;
    private boolean needInit = false; //true when the robot needs init on moving to next stage
    private int basePos = 0; //tick position of a full rotation before this iteration

    private final double gearRatio = 1.5;  //Motor rotations for geared rotation
    private final int encoderRotation = 1120; //number of encoder clicks per rotation

    private final double shootBlockDelta = 0.5; //servo up position //TODO find
    private final double shootBlockPos = 0.37; //base position of shooter block position //TODO find
    private final long shootBlockTime = 300; //time for servo up //TODO find
    private final double dotDotDotDelta = 0.02; //very small change between iterations of loop //TODO find

    private final int singleRotation = (int) (gearRatio * encoderRotation);
    private final int pullbackPosition =
            (int) ((300.0 / 360) * singleRotation); //Degrees of position when shooter is in pullback position //TODO find

    public enum ShootState {
        STOPPED, //stopped and ready to shoot
        PULLBACK, //first drive section to drive until almost ready to shoot
        LOAD, //servo allows ball in
        QUEUE_UP, //queue up next ball
        LOAD_RESET, //reset to base position
        COCKED_AND_LOADED, //next press of a will shoot a ball
        SHOOT //finishes shoot rotation
    }

    //Encoder Vars
    private final int inchDist = (int) (560 / 12.56); //ticks per inch (theoretically)
    private final double efficiencyMultiplier = 3/2.0; //roughly the efficiency of the drivetrain
    private final double fullRotationTicks = dist2ticks(56.52); //James's calculated ticks for rotation
    private final double rotationEfficiencyMultiplier = 1; //added efficiency multiplier for turning

    //timeDrive cached variables
    private double leftPower=0, rightPower=0;
    private long time=0;

    //distDrive cached variables
    private double leftStartPos = 0, rightStartPos = 0;
    private long ticks; //ticks to drive

    //drive2dist cached variables
    private double stopDist = 0;
    private boolean direction = true;



    public Robot(HardwareMap map)
    {
        hardwareInit(map);
        init();
    }

    protected void hardwareInit(HardwareMap map) {
        //Hardware
        //motors
        leftMotor = map.dcMotor.get("left");
        rightMotor = map.dcMotor.get("right");
        shooter = map.dcMotor.get("shooter");
        caroline = map.dcMotor.get("caroline");
        intake = map.dcMotor.get("intake");
        //servos
        beaconPusher = map.servo.get("beaconPusher");
        shooterBlock = map.servo.get("shooterBlock");
        //capHolder = map.servo.get("capHolder");

        shooterBlock.setPosition(shootBlockPos);

        //distance sensors
        I2cDeviceSynch rawFrontDist = map.i2cDeviceSynch.get("frontDist");
        frontDist = new ModernRoboticsI2cRangeSensor(rawFrontDist);
        frontDist.setI2cAddress(new I2cAddr(0x12));

        I2cDeviceSynch rawLeftFrontDist = map.i2cDeviceSynch.get("leftFrontDist");
        leftFrontDist = new ModernRoboticsI2cRangeSensor(rawLeftFrontDist);
        leftFrontDist.setI2cAddress(new I2cAddr(0x14));

        I2cDeviceSynch rawLeftRearDist = map.i2cDeviceSynch.get("leftRearDist");
        leftRearDist = new ModernRoboticsI2cRangeSensor(rawLeftRearDist);
        leftRearDist.setI2cAddress(new I2cAddr(0x16));

        //distance sensor readers //TODO this may replace others
//        frontDistReader = new I2cDeviceReader(rawFrontDist, new I2cAddr(0x12), 0x04, 2);
//        leftFrontDistReader = new I2cDeviceReader((I2cDevice)rawFrontDist, new I2cAddr(0x14), 0x04, 2);
//        leftRearDistReader = new I2cDeviceReader((I2cDevice)rawFrontDist, new I2cAddr(0x16), 0x04, 2);

        frontDist.getDistance(DistanceUnit.CM);

        //DEPRECATED sideFlipperMotor = map.dcMotor.get("flipper");
        //DEPRECATED beaconHitter = map.dcMotor.get("beacon");


        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        caroline.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //DEPRECATED beaconHitter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //DEPRECATED sideFlipperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void init() {
        lastStageTime = System.currentTimeMillis();

        //wall follow init
        errors = new LinkedList<>();
    }

    /**
     * Uses range sensor to find distance from front sensor
     * @return returns distance in cm detected by front sensor
     */

    //SENSORS-------------------------------------------------

    public double getFrontDist()
    {
        return frontDist.getDistance(DistanceUnit.CM);
    }

    public double getLeftFrontDist()
    {
        return leftFrontDist.getDistance(DistanceUnit.CM);
    }

    public double getLeftRearDist()
    {
        return leftRearDist.getDistance(DistanceUnit.CM);
    }



    //SHOOTER-------------------------------------------------

    //debug
    public int getShooterPos() {
        return shooter.getCurrentPosition();
    }

    @Deprecated
    public void setShooterPos(double degrees)
    {
        int currentPos = shooter.getCurrentPosition();
        int prevFullRotation = currentPos / (int)(gearRatio * encoderRotation); //truncating division to get prev

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int encoderClicks = prevFullRotation + (int) ((degrees / 360) * (gearRatio * encoderRotation));

        shooter.setTargetPosition(shooter.getCurrentPosition() - encoderClicks);
        shooter.setPower(1);
    }

    @Deprecated
    public void launch()
    {
        if (!shooterIsBusy()) {
            this.setShooterPos(360);
        }
    }

    public ShootState getShootState() {
        return shootState;
    }

    /**
     * Launches ball in two phases of pressing a (unless a is held)
     * @param act if true, it will start the launch or shoot (depending on state), else finish current one
     */

    public void launch(boolean act, Telemetry telemetry) {
        telemetry.addData("ShootState", shootState);
        switch(shootState) {
            case STOPPED:
                if(act) {
                    basePos = singleRotation * (shooter.getCurrentPosition() / singleRotation);
                        //finds previous rotation position using truncating division to find number of full rotations

                    shootState = ShootState.PULLBACK;
                    prevTime = System.currentTimeMillis();
                    needInit = true;
                }
                break;//problem after this block
                //continue to pullback if starting another launch
            case PULLBACK:
                if (needInit) {//skipping
                    shooter.setPower(0.9);
                    needInit = false;
                }

                if(shooter.getCurrentPosition() > basePos + pullbackPosition) {
                    //position is past base position plus position for loaded
                    shooter.setPower(0);
                    shootState = ShootState.LOAD;
                    prevTime = System.currentTimeMillis();
                    needInit = true;
                }
                break;
            case LOAD:
                if(needInit) {
                    shooterBlock.setPosition(shootBlockPos + shootBlockDelta);
                    needInit = false;
                }
                if (System.currentTimeMillis() > prevTime + shootBlockTime) {
                    shootState = ShootState.QUEUE_UP;
                    prevTime = System.currentTimeMillis();
                    needInit = true;
                }
                break;
            case QUEUE_UP:
                if (needInit) {
                    shooterBlock.setPosition(shootBlockPos - shootBlockDelta);
                    needInit = false;
                }
                if (System.currentTimeMillis() > prevTime + 2 * shootBlockTime) {
                    shootState = ShootState.LOAD_RESET;
                    prevTime = System.currentTimeMillis();
                    needInit = true;
                }
                break;
            case LOAD_RESET:
                if(needInit) {
                    needInit = false;
                }
                if (shooterBlock.getPosition() > shootBlockPos) {
                    shootState = ShootState.COCKED_AND_LOADED;
                    prevTime = System.currentTimeMillis();
                    needInit = true;
                } else {
                    shooterBlock.setPosition(shooterBlock.getPosition() + dotDotDotDelta);
                }
                break;
            case COCKED_AND_LOADED:
                if(act) {
                    shootState = ShootState.SHOOT;
                    prevTime = System.currentTimeMillis();
                    needInit = true;
                }
                break;
            case SHOOT:
                if(needInit) {
                    shooter.setPower(1);
                    needInit = false;
                }

                if (shooter.getCurrentPosition() > basePos + singleRotation) {
                    shooter.setPower(0);
                    shootState = ShootState.STOPPED;
                    prevTime = System.currentTimeMillis();
                    needInit = false;
                }
                break;

        }
    }


    public void liftBlock()
    {
        shooterBlock.setPosition(1);               //TODO ENSURE POSITION
    }

    public void lowerBlock()
    {
        shooterBlock.setPosition(0);               //TODO ENSURE POSITION
    }

    public void intake(double power)
    {
        intake.setPower(power);
    }

    public boolean shooterIsBusy()
    {
        return shooter.isBusy();
    }

    //TODO TODO TODO Put shoot and load algorithm here instead of `New TeleOp`

    //DRIVETRAIN----------------------------------------------

    public void setRightPower(double speed)
    {
        if(forward)
            rightMotor.setPower(Range.clip(speed, -1, 1));
        else
            leftMotor.setPower(Range.clip(-speed, -1, 1));
    }

    public void setLeftPower(double speed)
    {
        if(forward)
            leftMotor.setPower(Range.clip(speed, -1, 1));
        else
            rightMotor.setPower(Range.clip(-speed, -1, 1));
    }

    public void brake()
    {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void reverseFront()
    {
        forward = !forward;
    }

    public void setForward() {
        forward = true;
    }

    public void setBackward() {
        forward = false;
    }

    /**
     * Gets direction of bot
     * @return Returns true if going forward else false
     */
    public boolean getDirection() {
        return forward;
    }

    public boolean checkLeftEncoder() {
        if (leftMotor.isBusy()) {
            if (leftMotor.getCurrentPosition() == leftLastPos) {
                return false;
            }
            leftLastPos = leftMotor.getCurrentPosition();
        }
        return true;
    }

    public boolean checkRightEncoder() {
        if (rightMotor.isBusy()) {
            if (rightMotor.getCurrentPosition() == rightLastPos) {
                return false;
            }
            rightLastPos = rightMotor.getCurrentPosition();
        }
        return true;
    }

    public boolean checkShooterEncoder() {
        if (shooter.isBusy()) {
            if (shooter.getCurrentPosition() == shootLastPos) {
                return false;
            }
            shootLastPos = shooter.getCurrentPosition();
        }
        return true;
    }

    //BEACON--------------------------------------------------

    public void pushBeacon()
    {
        beaconPusher.setPosition(0);        //TODO ENSURE POSITION
    }
    public void retractBeacon()
    {
        beaconPusher.setPosition(1);        //TODO ENSURE POSITION
    }


    //LIFT----------------------------------------------------
    @Deprecated
    public void lift(double power)
    {
        caroline.setPower(power);
    }
    @Deprecated
    public void lower(double power)
    {
        caroline.setPower(-power);
    }
    @Deprecated
    public void liftBrake()
    {
        //caroline.setPower(0);
    }
    @Deprecated
    public void ballIn()
    {
        //capHolder.setPosition(1);           //TODO ENSURE POSITION
    }
    @Deprecated
    public void ballOut()
    {
        //capHolder.setPosition(0);           //TODO ENSURE POSITION
    }



    //TELEMETRY-----------------------------------------------

    public void logData(Telemetry telemetry) {
        telemetry.addData("FrontDist", getFrontDist());
    }



    //TELE-AUTONOMOUS-----------------------------------------
    /**
     * Follows wall using PID controller
     * @param kp Proportional constant
     * @param kd Differential constant
     * @param ki Integral constant
     * @param drivePower Ideal drve power (will be the average of left and right
     */
    public void wallFollow(double kp, double kd, double ki, double drivePower, double goal, Telemetry telemetry) {
        double realDist = Evil.distFromWall(leftDistSeparation, getLeftFrontDist(),getLeftRearDist());
        double error = realDist - goal;

        if(!errors.isEmpty()) {
            //We have enough information to actually wall follow
            double prevError = errors.getLast(); //get previous error for D part
            errors.addLast(error);
            if(errors.size() > errorsPeriod) {
                errors.removeFirst(); //Remove first to keep constant window size
            }
            double currTime = System.nanoTime() / 1e9; //current time for use in I and D parts
            double dt = currTime - prevNanoTime; //Time delta

            double p = kp* error; //proportional part
            double i = ki * sum(errors) * dt / errors.size(); //integral part
            double d = kd * (error - prevError) / dt; //differential part

            double steering = p + i + d;

            telemetry.addData("wf -> dist", realDist);
            telemetry.addData("wf -> raw power", drivePower);
            telemetry.addData("wf -> dt", dt);

            telemetry.addData("wf -> p", p);
            telemetry.addData("wf -> i", i);
            telemetry.addData("wf -> d", d);
            telemetry.addData("wf -> steering", steering);

            //If steering is positive go left (rightPower > leftPower) else go right (opposite)
            double leftPower = drivePower - steering;
            double rightPower = drivePower + steering;
            telemetry.addData("wf -> leftPower BC", leftPower);
            telemetry.addData("wf -> rightPower BC", rightPower);

            //clip drive to always go in the direction we want
            if(drivePower > 0) {
                leftPower = Range.clip(leftPower, 0, 1);
                rightPower = Range.clip(rightPower, 0, 1);
            } else {
                leftPower = Range.clip(leftPower, -1, 0);
                rightPower = Range.clip(rightPower, -1, 0);
            }
            telemetry.addData("wf -> leftPower", leftPower);
            telemetry.addData("wf -> rightPower", rightPower);

            setLeftPower(leftPower);
            setRightPower(rightPower);
        } else {
            errors.addLast(error);
        }
        prevNanoTime = System.nanoTime() / 1e9; //Resets prevTime in both cases for next call
    }

    private double sum(List<Double> list) {
        double total = 0;
        for(double item : list) {
            total += item;
        }
        return total;
    }

    /**
     * Resets wall follow errors list
     * Note: should be called whenever not using wall_follow and will do nothing if list is empty
     */
    public void resetWallFollow() {
        while(!errors.isEmpty()) {
            errors.removeFirst();
        }
    }

    /**
     * Converts angle to encoder ticks (only works if wheels drive same speed)
     *
     * @param angle Angle in degrees to convert
     * @return returns ticks of turn
     */
    public int angle2ticks(double angle) {
        return (int) (angle / 360 * fullRotationTicks / rotationEfficiencyMultiplier);
    }

    public int dist2ticks(double dist) {
        return (int) (dist * inchDist / efficiencyMultiplier);
    }


    /**
     * Drive a certain amount of encoder ticks with constant power
     *
     * @param leftPower  Left drive power (with camera end as front)
     * @param rightPower Right drive power (with camera end as front)
     * @param ticks      Number of encoder ticks to drive (on either wheel - which one gets there first)
     *                   Note: it uses the absolute value to include negative distances
     */
    public void distDriveTicks(double leftPower, double rightPower, long ticks) {
        if (!States.isBusy(state)) {
            setState(State.DIST_DRIVE); //Set state to start going with this op

            //initialize motors
            this.setLeftPower(leftPower);
            this.setRightPower(rightPower);

            //Cached vars
            this.leftPower = leftPower;
            this.rightPower = rightPower;
            this.leftStartPos = abs(leftMotor.getCurrentPosition());
            this.rightStartPos = abs(rightMotor.getCurrentPosition());
            this.ticks = ticks;
        }
        if (state == State.DIST_DRIVE) {
            if (abs(leftMotor.getCurrentPosition()) > leftStartPos + ticks
                    || abs(rightMotor.getCurrentPosition()) > rightStartPos + ticks) {
                //Distance is up - it's done driving
                cancel(); //call one function to stop everything instead of doing it myself
                setState(State.SUCCESS);
            }
            //continue driving
        }
    }

    public void setState(State state) {
        this.state = state;
        lastStageTime = System.currentTimeMillis();
    }

    /**
     * Cancels any action currently going on (mostly for emergencies)
     */
    public void cancel() {
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        state = State.CANCELLED;
        lastStageTime = System.currentTimeMillis();
        //TODO set everything to defaults
    }

    /**
     * Drive until frontDist is less than position
     *
     * @param leftPower  Left drive power (with camera end as front)
     * @param rightPower Right drive power (with camera end as front)
     * @param dist distance (cm) from any object at front
     * @param time max time to drive
     * @param direction if true, going forward, else going backward (i.e. object getting farther)
     */
    public void drive2dist(double leftPower, double rightPower, double dist, long time, boolean direction) {
        if(!States.isBusy(state)) {
            setState(State.DRIVE2DIST); //Set state to start going with this op

            //initialize motors
            this.setLeftPower(leftPower); //TODO check that these are in the same direction
            this.setRightPower(rightPower);

            //Cached vars
            this.leftPower = leftPower;
            this.rightPower = rightPower;
            this.stopDist = dist;
            this.time = time;
            this.direction = direction;
        }
        if(state == State.DRIVE2DIST) {
            if(System.currentTimeMillis() >= lastStageTime + time) {
                //Time's up - it's done driving
                cancel(); //call one function to stop everything instead of doing it myself
                setState(State.FAILURE_TIMEOUT);
            }
            if (direction ? (getFrontDist() <= dist) : (getFrontDist() >= dist)) {
                //distance less than for going forward or greater than for backward drive
                cancel();
                setState(State.SUCCESS);
            }
            //continue driving
        }
        //TODO can we just disregard bad ops?
    }

    public boolean isBusy() {
        return States.isBusy(state);
    }

    /**
     * Drive a certain amount of time with constant power
     *
     * @param leftPower  Left drive power (with camera end as front)
     * @param rightPower Right drive power (with camera end as front)
     * @param time
     */
    public void timeDrive(double leftPower, double rightPower, long time) {
        if(!States.isBusy(state)) {
            setState(State.TIME_DRIVE); //Set state to start going with this op

            //initialize motors
            this.setLeftPower(leftPower); //TODO check that these are in the same direction
            this.setRightPower(rightPower);

            //Cached vars
            this.leftPower = leftPower;
            this.rightPower = rightPower;
            this.time = time;
        }
        if(state == State.TIME_DRIVE) {
            if(System.currentTimeMillis() >= lastStageTime + time) {
                //Time's up - it's done driving
                cancel(); //call one function to stop everything instead of doing it myself
                setState(State.SUCCESS);
            }
            //continue driving
        }
        //TODO can we just disregard bad ops?
    }


    /**********************************************************************************************
     * \
     * DEPRECATED STUFF (DEPRECATED: SHIT)                                                        *
     * \
     **********************************************************************************************/

    @Deprecated
    public void stopShooter ()
    {
        shooter.setPower(0);
    }
    @Deprecated
    public void setShooterPower(double power)
    {
        shooter.setPower(Range.clip(power, -1, 1));
    }
    @Deprecated
    public double getDist() {
        return getFrontDist();
    }
    @Deprecated
    public void flipIn() {
        //sideFlipperMotor.setPower(.4);
    }
    @Deprecated
    public void flipOut() {
        //sideFlipperMotor.setPower(-.7);
    }
    @Deprecated
    public void stopFlipper() {
        //sideFlipperMotor.setPower(0);
    }
    @Deprecated
    public void shoot()
    {
        if(!shooter.isBusy()) {
            shooter.resetDeviceConfigurationForOpMode();
            shooter.setTargetPosition(1480);
        }
    }
}
