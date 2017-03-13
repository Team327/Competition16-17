package org.firstinspires.ftc.teamcode.experimental;

/**
 * A module for random stuff with an evil side
 * Created by roboticsteam on 3/13/2017.
 */

public class Evil {
    /**
     * Works
     */
    public static double invSqrt(double x) {
        double xhalf = 0.5d * x;
        long i = Double.doubleToLongBits(x); //casual bit level hacking
        i = 0x5fe6ec85e7de30daL - (i >> 1); //What the fuck
        x = Double.longBitsToDouble(i);//Why the fuck
        x *= (1.5d - xhalf * x * x);
        //x *= (1.5d - xhalf * x * x); //2nd iteration for accuracy
        return x; //how the fuck
    }
}
