package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by gssmrobotics on 1/14/2017.
 */

@Disabled
@Autonomous(name = "Red Corner Autonomoose")
public class RedSideAuto extends OpMode {
    Robot robot;

    boolean lastShooterFixerPositive = true;

    public enum State {NULL, DELAY, SHOOT, FORWARD_1, BACK_LEFT, BACK_RIGHT, STOP}

    State stage = State.NULL;

    public static final double TIME_DELAY = 8000;
    public static final double TIME_SHOOT = 1000;
    public static final double TIME_FORWARD_1 = 8000;
    public static final double TIME_BACK_LEFT = 400;
    public static final double TIME_BACK_RIGHT = 400;
    public static final double STRONGER_POWER = 1.0; //Power of stronger motor when turning
    public static final double WEAKER_POWER = 0.7; //Power of weaker motor when turning

    double lastStageTime = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.reverseFront();
        stage = State.SHOOT;
        lastStageTime = 0;
    }

    @Override
    public void loop() {
        if (lastStageTime == 0) {
            lastStageTime = System.currentTimeMillis();
        } else {
            switch (stage) {
                case DELAY:
                    if (System.currentTimeMillis() - lastStageTime < TIME_DELAY) {
                        robot.brake();
                    } else {
                        lastStageTime = System.currentTimeMillis();
                        stage = State.SHOOT;
                    }
                case SHOOT:
                    if (System.currentTimeMillis() - lastStageTime < TIME_SHOOT) {
                        robot.shoot();
                    } else {
                        robot.stopShooter();
                        stage = State.FORWARD_1;
                        lastStageTime = System.currentTimeMillis();
                    }
                    break;
                case FORWARD_1:
                    if (System.currentTimeMillis() - lastStageTime < TIME_FORWARD_1) {
                        robot.setLeftPower(1);
                        robot.setRightPower(1);
                    } else {
                        robot.brake();
                        stage = State.BACK_LEFT;
                        lastStageTime = System.currentTimeMillis();
                    }
                    break;
                case BACK_LEFT:
                    if (System.currentTimeMillis() - lastStageTime < TIME_BACK_LEFT) {
                        robot.setRightPower(-STRONGER_POWER);
                        robot.setLeftPower(-WEAKER_POWER);
                    } else {
                        stage = State.BACK_RIGHT;
                        lastStageTime = System.currentTimeMillis();
                    }
                    break;
                case BACK_RIGHT:
                    if (System.currentTimeMillis() - lastStageTime < TIME_BACK_RIGHT) {
                        robot.setRightPower(-WEAKER_POWER);
                        robot.setLeftPower(-STRONGER_POWER);
                    } else {
                        stage = State.STOP;
                        lastStageTime = System.currentTimeMillis();
                    }
                    break;
                case STOP:
                    robot.brake();
                    break;
                default:
                    telemetry.addData("ERROR", "DEFAULT STATEMENT REACHED");
                    break;
            }
        }
    }
}
