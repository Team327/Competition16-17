package org.firstinspires.ftc.teamcode.competition;

/**
 * Created by gssmrobotics on 11/17/2016.
 */

public class TeamConstants {
    public static final double TIRE_RADIUS = 7.520 / 2; // small tire (cm)
//    public static final double BEACON_CENTER = 0.54; //Beacon hitter servo center position
//    public static final double BEACON_HIT = 0.08; //Beacon hitter position change to hit
    public static final double FLIPPER_SPEED = 0.8;
    public static final int ANDYMARK_TICKS_PER_REV = 1120; //ticks per revolution
    public static final int BEACON_HIT_TICKS =100;//ticks to hit
    public static final double DIST_THRESHOLD = 0.036;

    //Wall Follow PID constants
    public static final double WF_Kp = 0.006;//increase until destructive oscillations ensue, move to d
    public static final double WF_Kd = 0.000;//increase until destructive oscillations are ended
    public static final double WF_Ki = 0.000;//No steady state error
}
