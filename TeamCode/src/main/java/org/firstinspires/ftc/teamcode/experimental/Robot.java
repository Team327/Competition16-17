package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by gssmrobotics on 1/17/2017.
 */



public class Robot {
    protected DcMotor leftMotor, rightMotor, shooter, caroline, intake;
    protected Boolean forward = true;
    protected ModernRoboticsI2cRangeSensor frontDist;
    protected Servo beaconPusher, shooterBlock, capHolder;

    public Robot(HardwareMap map)
    {
        //Hardware
            //motors
        leftMotor = map.dcMotor.get("left");
        rightMotor = map.dcMotor.get("right");
        shooter = map.dcMotor.get("shooter");
        frontDist = new ModernRoboticsI2cRangeSensor(map.i2cDeviceSynch.get("frontDist"));
        caroline = map.dcMotor.get("caroline");
        intake = map.dcMotor.get("intake");
            //servos
        beaconPusher = map.servo.get("beaconPusher");
        shooterBlock = map.servo.get("shooterBlock");
        capHolder = map.servo.get("capHolder");

            //DEPRECATED sideFlipperMotor = map.dcMotor.get("flipper");
            //DEPRECATED beaconHitter = map.dcMotor.get("beacon");


        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        caroline.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



            //DEPRECATED beaconHitter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //DEPRECATED sideFlipperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /**
     * Uses range sensor to find distance from front sensor
     * @return returns distance in cm detected by front sensor
     */

    public double getFrontDist()
    {
        return frontDist.getDistance(DistanceUnit.CM);
    }



    //SHOOTER

    public void stopShooter ()
    {
        shooter.setPower(0);
    }


    public void setShooterPower(double power)
    {
        shooter.setPower(power);
    }

    public void liftBlock()
    {

    }

    public void lowerBlock()
    {

    }

    //DRIVETRAIN

    public void setRightPower(double speed)
    {
        if(forward)
            rightMotor.setPower(speed);
        else
            leftMotor.setPower(speed);
    }

    public void setLeftPower(double speed)
    {
        if(forward)
            leftMotor.setPower(-speed);
        else
            rightMotor.setPower(-speed);
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



    //BEACON

    public void pushBeacon()
    {
        beaconPusher.setPosition(0);

    }

    public void retractBeacon()
    {
        beaconPusher.setPosition(1);
    }



    //TELEMETRY

    public void logData(Telemetry telemetry) {
        telemetry.addData("FrontDist", getFrontDist());
    }



    //DEPRECATED

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
