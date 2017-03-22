package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    private final double gearRatio = 1.5;  //Motor rotations for geared rotation
    private final double encoderRotation = 1120; //number of encoder clicks per rotation

    private int leftLastPos = 0;
    private int rightLastPos = 0;
    private int shootLastPos = 0;



    public Robot(HardwareMap map)
    {
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

            //distance sensors
        frontDist = new ModernRoboticsI2cRangeSensor(map.i2cDeviceSynch.get("frontDist"));
        frontDist = new ModernRoboticsI2cRangeSensor(map.i2cDeviceSynch.get("leftFrontDist"));
        frontDist = new ModernRoboticsI2cRangeSensor(map.i2cDeviceSynch.get("leftRearDist"));

        //DEPRECATED sideFlipperMotor = map.dcMotor.get("flipper");
            //DEPRECATED beaconHitter = map.dcMotor.get("beacon");


        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        caroline.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //wall follow init
        errors = new LinkedList<>();

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


    public void setShooterPos(double degrees)
    {
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int encoderClicks = (int) ((degrees / 360) * (gearRatio * encoderRotation));

        shooter.setTargetPosition(shooter.getCurrentPosition() - encoderClicks);
        shooter.setPower(1);
    }


    public void launch()
    {

        if (!shooterIsBusy()) {
            this.setShooterPos(360);
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
     * DEPRECATED SHIT                                          *
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
