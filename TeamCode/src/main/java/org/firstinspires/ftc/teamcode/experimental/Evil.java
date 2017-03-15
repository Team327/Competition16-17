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
        long i = Double.doubleToLongBits(x); //casual bit level hacking time
        i = 0x5fe6ec85e7de30daL - (i >> 1); //What the fuck
        x = Double.longBitsToDouble(i);//Why the fuck
        x *= (1.5d - xhalf * x * x);
        //x *= (1.5d - xhalf * x * x); //2nd iteration for accuracy
        return x; //how the fuck
    }

    /**
     * Gives the distance of the front sensor to the wall at the closest point
     * @param sensorSeperation distance between the sensors
     * @param frontDist distance the front sensor registers
     * @param backDist distance the back sensor registers
     * @return Distance from the wall in the units they were given in
     */
    public static double distFromWall(double sensorSeperation,double frontDist, double backDist)
    {
        return sensorSeperation*frontDist*invSqrt(sensorSeperation*sensorSeperation+
                (backDist-frontDist)*(backDist-frontDist));
    }
}
