package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by gssmrobotics on 1/14/2017.
 */

@Autonomous(name = "Basic Autonomoose")
public class BasicAuto extends OpMode {
    DcMotor left = null, right = null, shooter = null;

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
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        shooter = hardwareMap.dcMotor.get("shooter");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        stage = State.DELAY;
        lastStageTime = System.currentTimeMillis();
        left.setPower(0);
        right.setPower(0);
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
                    left.setPower(1);
                    right.setPower(1);
                } else {
                    left.setPower(0);
                    right.setPower(0);
                    stage = State.SHOOTER_LOAD;
                    lastStageTime = System.currentTimeMillis();
                }
                break;
            case SHOOTER_LOAD:
                if(System.currentTimeMillis() - lastStageTime < TIME_SHOOT_LOAD) {
                    shooter.setPower(-1);
                } else {
                    shooter.setPower(0);
                    stage = State.SHOOT;
                    lastStageTime = System.currentTimeMillis();
                }
            case SHOOT:
                if(System.currentTimeMillis() - lastStageTime < TIME_SHOOT) {
                    shooter.setPower(1);
                } else {
                    shooter.setPower(0);
                    stage = State.TO_BALL;
                    lastStageTime = System.currentTimeMillis();
                }
            case TO_BALL:
                if(System.currentTimeMillis() - lastStageTime < TIME_TO_BALL) {
                    left.setPower(1);
                    right.setPower(1);
                } else {
                    left.setPower(0);
                    right.setPower(0);
                    stage = State.DONE;
                    lastStageTime = System.currentTimeMillis();
                }

            case NULL:
            default:
                //Do nothing
                break;
        }
    }
}
