package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by gssmrobotics on 1/14/2017.
 */
@Autonomous(name = "DriveShootRedAuto")
public class DriveShootRedAuto extends OpMode {
    Robot robot;

    public enum State {DRIVE,SHOOT,BUMP, ONWARD,WAIT, TURN,UPWARD, SHORT_WAIT, DONE}
    State stage;

    double lastStageTime = 0;

    boolean communism = false; //true if red

    @Override
    public void init() {
        try {
            robot = new Robot(hardwareMap);
        } catch(Exception e) {
            telemetry.addData("ERROR ERROR ERROR", "Unable to connect to robot");
            robot = new SimBot(hardwareMap, this, gamepad2);
            //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
            //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
            //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
            //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
            //TODO remove this - it's a super security concern
        }
        stage = State.DRIVE;
        lastStageTime=0;

        robot.init();

        if(communism) {
            robot.setBackward();
        }
    }

    @Override
    public void loop() {
        if(lastStageTime==0) {
            lastStageTime = System.currentTimeMillis();
        } else {
            switch (stage) {
                case DRIVE:
                    robot.timeDrive(0.5, 0.5, 1500);
                    if(!robot.isBusy())
                    {
                        lastStageTime=System.currentTimeMillis();
                        stage= State.SHOOT;
                        robot.cancel();
                    }
                    break;
                case SHORT_WAIT:
                    if(System.currentTimeMillis() > lastStageTime + 500) {
                        lastStageTime = System.currentTimeMillis();
                        stage = State.SHOOT;
                        robot.cancel();
                    }
                case SHOOT:
                    robot.launch(true,telemetry);
                    if(System.currentTimeMillis()-lastStageTime>5000)
                    {
                        lastStageTime=System.currentTimeMillis();
                        stage= State.DONE; //Set to ONWARD if you want to do experimental stuff
                        robot.cancel();
                    }
                    break;
                //Note that these parts are optional and will be skipped
                case ONWARD:
                    robot.timeDrive(0.5, 0.5, 300);
                    if(!robot.isBusy())
                    {
                        lastStageTime=System.currentTimeMillis();
                        stage= State.TURN;
                        robot.cancel();
                    }
                    break;
                case TURN:
                    robot.timeDrive(-0.5, 0.5, 400);
                    if(!robot.isBusy()) {
                        lastStageTime =System.currentTimeMillis();
                        stage = State.WAIT;
                        robot.cancel();
                    }
                    break;
                case WAIT:
                    if(System.currentTimeMillis() > lastStageTime+4000) {
                        lastStageTime=System.currentTimeMillis();
                        stage= State.UPWARD;
                        robot.cancel();
                    }
                    break;
                case UPWARD:
                    robot.timeDrive(0.5, 0.5, 2000);
                    if(!robot.isBusy()) {
                        lastStageTime = System.currentTimeMillis();
                        stage = State.DONE;
                        robot.cancel();
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
