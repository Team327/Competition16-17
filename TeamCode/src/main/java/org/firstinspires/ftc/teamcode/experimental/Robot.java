package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.LinkedList;
import java.util.List;

/**
 * Created by gssmrobotics on 1/17/2017.
 */



public class Robot {
    protected DcMotor leftMotor, rightMotor, shooter, caroline, intake;
    protected Boolean forward = true;
    protected ModernRoboticsI2cRangeSensor frontDist, leftFrontDist, leftRearDist;
    protected Servo beaconPusher, shooterBlock, capHolder;

    //wall follow cached variables
    private LinkedList<Double> errors; //sliding window of errors
    private double prevTime = 0;
    private final int errorsPeriod = 10; //period of sliding window for error (used in I)
    private final double goal = 4; //Goal distance in CM
    private final double leftDistSeparation = 30; //TODO measure distance between sensors on bot

    private int leftLastPos = 0;
    private int rightLastPos = 0;
    private int shootLastPos = 0;

    //Shooting variables
    private ShootState shootState = ShootState.STOPPED;
    private long shootPrevTime = 0;
    private boolean needInit = false; //true when the robot needs init on moving to next stage
    private int basePos = 0; //tick position of a full rotation before this iteration

    private final double gearRatio = 1.5;  //Motor rotations for geared rotation
    private final int encoderRotation = 1120; //number of encoder clicks per rotation

    private final double shootBlockDelta = 0.5; //servo up position //TODO find
    private final double shootBlockPos = 0.5; //base position of shooter block position //TODO find
    private final long shootBlockTime = 500; //time for servo up //TODO find
    private final double dotDotDotDelta = 0.005; //very small change between iterations of loop //TODO find

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

    public Robot(HardwareMap map)
    {
        hardwareInit(map);

        //wall follow init
        errors = new LinkedList<>();
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
        capHolder = map.servo.get("capHolder");

        shooterBlock.setPosition(shootBlockPos);

        //distance sensors
        frontDist = new ModernRoboticsI2cRangeSensor(map.i2cDeviceSynch.get("frontDist"));
        leftFrontDist = new ModernRoboticsI2cRangeSensor(map.i2cDeviceSynch.get("leftFrontDist"));
        leftRearDist = new ModernRoboticsI2cRangeSensor(map.i2cDeviceSynch.get("leftRearDist"));

        //DEPRECATED sideFlipperMotor = map.dcMotor.get("flipper");
        //DEPRECATED beaconHitter = map.dcMotor.get("beacon");


        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        caroline.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //DEPRECATED beaconHitter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //DEPRECATED sideFlipperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        //telemetry.addData("launch breakpoint", 1);
        telemetry.addData("ShootState", shootState);
        switch(shootState) {
            case STOPPED:
                telemetry.addData("launch breakpoint", 2);

                telemetry.addData("motor power", shooter.getPower());//after this
                if(act) {
                    telemetry.addData("launch breakpoint", 3);
                    basePos = singleRotation * (shooter.getCurrentPosition() / singleRotation);
                        //finds previous rotation position using truncating division to find number of full rotations

                    shootState = ShootState.PULLBACK;
                    prevTime = System.currentTimeMillis();
                    needInit = true;
                }
                break;//problem after this block
                //continue to pullback if starting another launch
            case PULLBACK:
                telemetry.addData("launch breakpoint", 4);
                telemetry.addData("shooter power", shooter.getPower());

                if (needInit) {//skipping
                    telemetry.addData("launch breakpoint", 5);
                    shooter.setPower(0.9);
                    needInit = false;
                }

                telemetry.addData("shooter pos", shooter.getCurrentPosition());
                telemetry.addData("shooter goal pos", basePos + pullbackPosition);

                if(shooter.getCurrentPosition() > basePos + pullbackPosition) {
                    telemetry.addData("launch breakpoint", 6);
                    //position is past base position plus position for loaded
                    shooter.setPower(0);
                    shootState = ShootState.LOAD;
                    prevTime = System.currentTimeMillis();
                    needInit = true;
                }
                break;
            case LOAD:
                telemetry.addData("launch breakpoint", 7);
                if(needInit) {
                    telemetry.addData("launch breakpoint", 8);
                    shooterBlock.setPosition(shootBlockPos + shootBlockDelta);
                    needInit = false;
                }
                if (System.currentTimeMillis() > prevTime + shootBlockTime) {
                    telemetry.addData("launch breakpoint", 9);
                    shootState = ShootState.QUEUE_UP;
                    prevTime = System.currentTimeMillis();
                    needInit = true;
                }
                break;
            case QUEUE_UP:
                telemetry.addData("launch breakpoint", 9.1);
                if (needInit) {
                    telemetry.addData("launch breakpoint", 9.2);
                    shooterBlock.setPosition(shootBlockPos - shootBlockDelta);
                    needInit = false;
                }
                if (System.currentTimeMillis() > prevTime + 2 * shootBlockTime) {
                    telemetry.addData("launch breakpoint", 9.3);
                    shootState = ShootState.LOAD_RESET;
                    prevTime = System.currentTimeMillis();
                    needInit = true;
                }
                break;
            case LOAD_RESET:
                telemetry.addData("launch breakpoint", 9.5);
                if(needInit) {
                    telemetry.addData("launch breakpoint", 10);
                    needInit = false;
                }
                if (shooterBlock.getPosition() > shootBlockPos) {
                    telemetry.addData("launch breakpoing", 11);
                    shootState = ShootState.COCKED_AND_LOADED;
                    prevTime = System.currentTimeMillis();
                    needInit = true;
                } else {
                    shooterBlock.setPosition(shooterBlock.getPosition() + dotDotDotDelta);
                }
                break;
            case COCKED_AND_LOADED:
                telemetry.addData("launch breakpoint", 12);
                if(act) {
                    telemetry.addData("launch breakpoint", 13);
                    shootState = ShootState.SHOOT;
                    prevTime = System.currentTimeMillis();
                    needInit = true;
                }
                break;
            case SHOOT:
                if(needInit) {
                    telemetry.addData("launch breakpoint", 14);
                    shooter.setPower(1);
                    needInit = false;
                }

                telemetry.addData("shooter pos", shooter.getCurrentPosition());
                telemetry.addData("shooter goal pos", basePos + pullbackPosition);

                if (shooter.getCurrentPosition() > basePos + singleRotation) {
                    telemetry.addData("launch breakpoint", 15);
                    shooter.setPower(0);
                    shootState = ShootState.STOPPED;
                    prevTime = System.currentTimeMillis();
                    needInit = false;
                }
                break;

        }
        //telemetry.addData("launch breakpoint", 16);
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
            rightMotor.setPower(Range.clip(-speed, -1, 1));
        else
            leftMotor.setPower(Range.clip(-speed, -1, 1));
    }

    public void setLeftPower(double speed)
    {
        if(forward)
            leftMotor.setPower(Range.clip(speed, -1, 1));
        else
            rightMotor.setPower(Range.clip(speed, -1, 1));
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
        beaconPusher.setPosition(.75);        //TODO ENSURE POSITION
    }


    //LIFT----------------------------------------------------

    public void lift(double power)
    {
        caroline.setPower(power);
    }

    public void lower(double power)
    {
        caroline.setPower(-power);
    }

    public void ballIn()
    {
        capHolder.setPosition(1);           //TODO ENSURE POSITION
    }
    public void ballOut()
    {
        capHolder.setPosition(0);           //TODO ENSURE POSITION
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
    public void wallFollow(double kp, double kd, double ki, double drivePower, Telemetry telemetry) {
        telemetry.addData("wf breakpoint", 1); telemetry.update();
        double realDist = Evil.distFromWall(leftDistSeparation, getLeftFrontDist(), getLeftRearDist());
        telemetry.addData("wf breakpoint", 2); telemetry.update();
        double error = realDist - goal;
        telemetry.addData("wf breakpoint", 3); telemetry.update();
        if(!errors.isEmpty()) {
            telemetry.addData("wf breakpoint", 4); telemetry.update();
            //We have enough information to actually wall follow
            double prevError = errors.getLast(); //get previous error for D part
            errors.addLast(error);
            if(errors.size() > errorsPeriod) {
                errors.removeFirst(); //Remove first to keep constant window size
            }
            telemetry.addData("wf breakpoint", 5); telemetry.update();
            double currTime = System.currentTimeMillis(); //current time for use in I and D parts
            double dt = currTime - prevTime; //Time delta

            double p = error; //proportional part
            telemetry.addData("wf breakpoint", 6); telemetry.update();
            double i = sum(errors) * dt / errors.size(); //integral part
            telemetry.addData("wf breakpoint", 7); telemetry.update();
            double d = (error - prevError) / dt; //differential part

            double steering = p + i + d;

            //If steering is positive go left (rightPower > leftPower) else go right (opposite)
            double leftPower = drivePower - steering;
            double rightPower = drivePower + steering;

            telemetry.addData("wf breakpoint", 8); telemetry.update();
            setLeftPower(leftPower);
            setRightPower(rightPower);
            telemetry.addData("wf breakpoint", 9); telemetry.update();
            //TODO check if either goes out of bound so it doesn't just lose power to one side (maybe not necessary)
        } else {
            errors.addLast(error);
        }
        prevTime = System.currentTimeMillis(); //Resets prevTime in both cases for next call
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
