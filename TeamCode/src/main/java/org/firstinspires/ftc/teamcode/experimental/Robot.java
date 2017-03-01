package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by gssmrobotics on 1/17/2017.
 */



public class Robot {
    protected DcMotor leftMotor, rightMotor, shooter, sideFlipperMotor, beaconHitter;
    protected OpticalDistanceSensor dist;
    protected Boolean forward = true;

    public Robot(HardwareMap map)
    {
        leftMotor = map.dcMotor.get("left");
        rightMotor = map.dcMotor.get("right");
        shooter = map.dcMotor.get("shooter");
        sideFlipperMotor = map.dcMotor.get("flipper");
        beaconHitter = map.dcMotor.get("beacon");

        beaconHitter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sideFlipperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dist = map.opticalDistanceSensor.get("dist");

        dist.enableLed(true);
    }

    public double getDist()
    {
        return dist.getLightDetected();
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
}
