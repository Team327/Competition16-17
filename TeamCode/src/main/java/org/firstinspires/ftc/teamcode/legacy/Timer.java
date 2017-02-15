package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by gssmrobotics on 11/21/2016.
 */
@Disabled
public class Timer {

    private long initTime;
    private long tempTime;
    private long tempTime2;
    private long finalTime;

    public void start()
    {
        this.initTime = System.currentTimeMillis();

    }

    public void stop()
    {
        this.finalTime = System.currentTimeMillis();
    }

    public long get()
    {
        this.tempTime = System.currentTimeMillis();
        return (tempTime - initTime);
    }

    public int getS()
    {
        return (int) (this.get()/1000);
    }

    public void reset()
    {
        this.initTime = 0;
    }

    public void delay(long milliSeconds)
    {
        this.tempTime = System.currentTimeMillis();
        this.tempTime2 = tempTime;
        while((tempTime2 - tempTime) < milliSeconds)
        {
            tempTime2 = System.currentTimeMillis();
        }
    }
}
