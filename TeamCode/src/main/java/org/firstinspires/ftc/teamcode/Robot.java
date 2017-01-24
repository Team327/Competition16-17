package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by gssmrobotics on 1/17/2017.
 */



public class Robot {
    private DcMotor leftMotor,rightMotor,shooter;
    private Servo beaconHitter;
    private OpticalDistanceSensor leftDistance,rightDistance;
    private Boolean forward = true;

    public Robot(HardwareMap map)
    {
        leftMotor = map.dcMotor.get("left");
        rightMotor = map.dcMotor.get("right");
        shooter = map.dcMotor.get("shooter");

        beaconHitter = map.servo.get("beacon");

        leftDistance = map.opticalDistanceSensor.get("left");
        rightDistance = map.opticalDistanceSensor.get("right");

        rightDistance.enableLed(true);
        leftDistance.enableLed(true);
    }

    public double rightDist()
    {
        return rightDistance.getLightDetected();
    }

    public double leftDist()
    {
        return leftDistance.getLightDetected();
    }

    public void hitRightBeacon()
    {
        beaconHitter.setPosition(TeamConstants.BEACON_CENTER + TeamConstants.BEACON_HIT);
    }

    public void hitLeftBeacon ()
    {
        beaconHitter.setPosition(TeamConstants.BEACON_CENTER - TeamConstants.BEACON_HIT);
    }

    public void centerBeacon ()
    {
        beaconHitter.setPosition(TeamConstants.BEACON_CENTER);
    }

    /**
     * Slightly changes beacon position
     * @param correction Amount to change position by (positive is more {TODO which direction is this?}
     */
    public void correctBeacon (double correction) {
        beaconHitter.setPosition(Range.clip(beaconHitter.getPosition() + correction,
                TeamConstants.BEACON_CENTER - TeamConstants.BEACON_HIT,
                TeamConstants.BEACON_CENTER + TeamConstants.BEACON_HIT));
    }

    public double beaconPosition () {
        return beaconHitter.getPosition();
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
}
