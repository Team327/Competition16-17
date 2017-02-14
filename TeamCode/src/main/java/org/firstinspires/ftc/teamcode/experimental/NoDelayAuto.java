package org.firstinspires.ftc.teamcode.tournament;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by gssmrobotics on 1/14/2017.
 */

@Autonomous(name = "No Delay Autonomoose")
public class NoDelayAuto extends OpMode {
    Robot robot;

    boolean lastShooterFixerPositive = true;

    public enum State {NULL, DELAY, SHOOT, SAVING_PRIVATE_SHOOTER, TO_BALL, DONE}

    State stage = State.NULL;

    public static final double TIME_DELAY = 3000; //Wait 3 seconds to go
    public static final double TIME_SAVE_SHOOTER = 7000;
    public static final double TIME_SHOOT = 1000; //Time to shoot (ms)
    public static final double TIME_TO_BALL = 1425; //Distance to go to ball

    double lastStageTime = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.reverseFront();
        stage = State.DELAY;
        lastStageTime = 0;
    }

    @Override
    public void loop() {
        if (lastStageTime == 0) {
            lastStageTime = System.currentTimeMillis();
        } else {
            switch (stage) {
                case DELAY:
                    if (System.currentTimeMillis() - lastStageTime >= TIME_DELAY) {
                        stage = State.SHOOT;
                        lastStageTime = System.currentTimeMillis();
                    }
                    break;
                case SHOOT:
                    if (System.currentTimeMillis() - lastStageTime < TIME_SHOOT) {
                        robot.shoot();
                    } else {
                        robot.stopShooter();
                        stage = State.SAVING_PRIVATE_SHOOTER;
                        lastStageTime = System.currentTimeMillis();
                    }
                    break;
                case SAVING_PRIVATE_SHOOTER:
                    if (System.currentTimeMillis() - lastStageTime < TIME_SAVE_SHOOTER) {

                        robot.setShooterPower(lastShooterFixerPositive ? -.1 : .12);
                        lastShooterFixerPositive = !lastShooterFixerPositive;
                    } else {
                        robot.stopShooter();
                        stage = State.TO_BALL;
                        lastStageTime = System.currentTimeMillis();
                    }
                    break;
                case TO_BALL:
                    if (System.currentTimeMillis() - lastStageTime < TIME_TO_BALL) {
                        robot.setLeftPower(1);
                        robot.setRightPower(1);
                    } else {
                        robot.brake();
                        stage = State.DONE;
                        lastStageTime = System.currentTimeMillis();
                        telemetry.addData("Status", "You did the thing, congrats");
                    }
                    break;
                case DONE:

                    break;
                default:
                    //Should never be reached
                    break;
            }
        }
    }
}
