package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by gssmrobotics on 1/17/2017.
 */



public class Robot {
    protected DcMotor leftMotor, rightMotor, shooter, sideFlipperMotor, beaconHitter;
    protected Boolean forward = true;
    protected ModernRoboticsI2cRangeSensor frontDist;

    public Robot(HardwareMap map)
    {
        leftMotor = map.dcMotor.get("left");
        rightMotor = map.dcMotor.get("right");
        shooter = map.dcMotor.get("shooter");
        sideFlipperMotor = map.dcMotor.get("flipper");
        beaconHitter = map.dcMotor.get("beacon");
        frontDist = (ModernRoboticsI2cRangeSensor) map.i2cDeviceSynch.get("frontDist");

        beaconHitter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sideFlipperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Deprecated
    public double getDist()
    {
        return getFrontDist();
    }

    /**
     * Uses range sensor to find distance from front sensor
     * @return returns distance in cm detected by front sensor
     */
    public double getFrontDist()
    {
        return frontDist.getDistance(DistanceUnit.CM);
    }

    @Deprecated
    public void hitRightBeacon()
    {
        beaconHitter.setPower(.325);
    }

    @Deprecated
    public void hitLeftBeacon ()
    {
        beaconHitter.setPower(-.325);
    }

    @Deprecated
    public void centerBeacon ()
    {
        beaconHitter.setPower(0);
    }

    @Deprecated
    public double getBeaconPosition () {
        return beaconHitter.getCurrentPosition();
    }

    public void shoot()
    {
        shooter.setPower(1);
    }

    public void reverseShoot()
    {
        shooter.setPower(-1);
    }

    public void stopShooter ()
    {
        shooter.setPower(0);
    }

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

    public void flipIn() {
        sideFlipperMotor.setPower(.4);
    }

    public void flipOut() {
        sideFlipperMotor.setPower(-.7);
    }

    public void stopFlipper() {
        sideFlipperMotor.setPower(0);
    }

    public void setShooterPower(double power)
    {
        shooter.setPower(power);
    }

    public void logData(Telemetry telemetry) {
        telemetry.addData("FrontDist", getFrontDist());
    }
}
