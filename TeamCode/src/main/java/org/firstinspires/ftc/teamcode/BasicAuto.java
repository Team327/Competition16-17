package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by gssmrobotics on 1/14/2017.
 */

@Autonomous(name = "Basic Autonomoose")
public class BasicAuto extends OpMode {
    Robot robot;

    public enum State {NULL, DELAY, TO_SHOOT, SHOOTER_LOAD, SHOOT, TO_BALL, DONE}
    State stage = State.NULL;

    public static final double TIME_DELAY = 10000; //Wait 10 seconds to go
    public static final double TIME_SHOOT_PREPARE = 800; //Distance to go forward before shooting
    public static final double TIME_SHOOT_LOAD = 100; //Time to go backwards before shooting (ms)
    public static final double TIME_SHOOT = 1000; //Time to shoot (ms)
    public static final double TIME_TO_BALL = 2000; //Distance to go to ball

    double lastStageTime = 0;

    //TODO Fix everything (especially driving while shooting

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.reverseFront();
        stage = State.DELAY;
        lastStageTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        switch (stage) {
            case DELAY:
                if(System.currentTimeMillis() - lastStageTime >= TIME_DELAY) {
                    stage = State.TO_SHOOT;
                    lastStageTime = System.currentTimeMillis();
                }
                break;
            case TO_SHOOT:
                if(System.currentTimeMillis() - lastStageTime < TIME_SHOOT_PREPARE) {
                    robot.setRightPower(1);
                    robot.setLeftPower(1);
                } else {
                    robot.brake();
                    stage = State.SHOOTER_LOAD;
                    lastStageTime = System.currentTimeMillis();
                }
                break;
            case SHOOTER_LOAD:
                if(System.currentTimeMillis() - lastStageTime < TIME_SHOOT_LOAD) {
                    robot.reverseShoot();
                } else {
                    robot.stopShooter();
                    stage = State.SHOOT;
                    lastStageTime = System.currentTimeMillis();
                }
            case SHOOT:
                if(System.currentTimeMillis() - lastStageTime < TIME_SHOOT) {
                    robot.shoot();
                } else {
                    robot.stopShooter();
                    stage = State.TO_BALL;
                    lastStageTime = System.currentTimeMillis();
                }
            case TO_BALL:
                if(System.currentTimeMillis() - lastStageTime < TIME_TO_BALL) {
                    robot.setLeftPower(1);
                    robot.setRightPower(1);
                } else {
                    robot.brake();
                    stage = State.DONE;
                    lastStageTime = System.currentTimeMillis();
                }

            case DONE:

            default:
                //Should never be reached
                break;
        }
    }
}
