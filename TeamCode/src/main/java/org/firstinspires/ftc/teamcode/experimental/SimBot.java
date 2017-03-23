package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.lasarobotics.vision.opmode.VisionOpMode;

/**
 * Created by gssmrobotics on 3/23/2017.
 */

public class SimBot extends VisionRobot {
    private Gamepad pad;
    private int shooterPos=0, leftPos=0, rightPos=0;
    private double leftPower=0, rightPower=0;
    private boolean forward=true;
    private boolean hitterOut=false;
    private Telemetry out;

    private double baseDist = 60;
    private int rotorTicks = 1120;

    public SimBot(HardwareMap map, VisionOpMode op, Gamepad gamepad) {
        super(map, op);
        out = op.telemetry;
    }

    protected void hardwareInit(HardwareMap map) {
        //Do nothing
    }

    public double getFrontDist() {
        return baseDist * (1 - pad.right_trigger);
    }

    public double getLeftFrontDist() {
        return baseDist * (1 - pad.left_trigger);
    }

    public double getLeftRearDist() {
        return baseDist * (1 - pad.left_stick_y);
    }

    public int getShooterPos() {
        return shooterPos;
    }

    public void launch(boolean act, Telemetry telemetry) {
        shooterPos += rotorTicks * 3 / 2;
    }

    public void intake(double power) {
        //No sim necessary
    }

    public boolean shooterIsBusy() {
        return false; //shooting is automatic
    }

    public void setRightPower(double speed) {
        rightPower = speed;
    }

    public void setLeftPower(double speed) {
        leftPower = speed;
    }

    public void brake() {
        leftPower = 0;
        rightPower = 0;
    }

    public void reverseFront() {
        forward = !forward;
    }

    public void setForward() {
        forward = true;
    }

    public void setBackward() {
        forward = false;
    }

    public void pushBeacon() {
        hitterOut = true;
    }

    public void retractBeacon() {
        hitterOut = false;
    }

    public void lift(double power) {
        //Do nothing for now
    }

    public void lower(double power) {
        //Dot nothing for now
    }

    public void ballIn() {
        //Do nothing for now
    }

    public void ballOut() {
        //Do nothing for now
    }

    public void update() {
        leftPos += rotorTicks * leftPower * (forward ? 1 : -1);
        rightPos += rotorTicks * rightPower * (forward ? 1 : -1);
    }
}
