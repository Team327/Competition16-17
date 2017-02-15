package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


@Disabled
@Autonomous(name = "BeaconTester")

public class BeaconHitter extends OpMode {

    private double dist1Last,dist2Last;
    private OpticalDistanceSensor dist1 = null , dist2 = null;
    private Servo servo1 = null;
    private double lastError=0;
    private long lastTime= System.nanoTime();

    private enum STATE {ALIGNING,PRESSING,DONE}
    private STATE currState=STATE.ALIGNING;

    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");

        //Instantiate the sensor and servos
        servo1 = hardwareMap.servo.get("servo1");
        dist1 = hardwareMap.opticalDistanceSensor.get("dist1");
        dist2 = hardwareMap.opticalDistanceSensor.get("dist2");

        //Initialize sensors and servo
        servo1.setPosition(.5);
        dist1.enableLed(true);
        dist2.enableLed(true);
    }


    @Override
    public void loop()
    {
        switch(currState)
        {
            case ALIGNING:
                if(align()) currState=STATE.PRESSING;
                break;
            case PRESSING:
                break;
            case DONE:
                break;
        }


    }

    public boolean align()
    {
        double lum1 = dist1.getRawLightDetected()/dist1.getRawLightDetectedMax();
        double lum2 = dist2.getRawLightDetected()/dist2.getRawLightDetectedMax();
        double position = servo1.getPosition();

        double error = lum2-lum1;
        long thisTime = System.nanoTime();

        double Kp = .05;
        double correction = Kp*error + Kp*(lastError-error)/(lastTime-thisTime);
        lastTime=thisTime;
        lastError=error;
        if((position<.2 && correction > 0) || (position>.8 && correction <0) || (position <=.8 && position >= .2)) {
            servo1.setPosition(correction+position);
        }
        telemetry.addData("Status", "Dist1: " + lum1 + " Dist2: " + lum2);
        dist1Last=lum1;
        dist2Last=lum2;
        return equal(lum1,lum2,.001) && equal(lum1,.0694,.001);
    }

    public boolean press()
    {
        telemetry.addData("Status", "DONE. Dist1: " + dist1Last + " Dist2: " + dist2Last);
        return true;
    }

    public boolean equal(double a, double b,double tolerance)
    {
        return Math.abs(a-b)<tolerance;
    }
}
