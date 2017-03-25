package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by gssmrobotics on 3/23/2017.
 */

public class DefenseAuto extends OpMode {
    Robot robot;

    //telemetry
    private Telemetry.Item status;

    public enum State {
        DRIVE,
        SHORT_WAIT,
        SHOOT,
        DRIVE_UP,
        TURN_TO_BEACON,
        DRIVE_TO_BEACON,
        ALIGN_BEACONS,
        WALL_FOLLOW,
        DONE
    }
    State stage;

    //Time
    private long time = 0;
    private long lastStageTime =0;

    //single iteration variable
    private final boolean communism = true; //Are we the red alliance
    private final int balls2shoot = 2; //Number of balls to shoot


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

        if(time ==0)
        {
            time = System
        }
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
                    for (int i = 1; i <= balls2shoot; i++) {
                        while (robot.getShootState() != Robot.ShootState.COCKED_AND_LOADED) {
                            //Start shooting a ball (get to COCKED_AND_READY state)
                            telemetry.addData("auto status", "shooting ball " + i);
                            telemetry.addData("bot status", robot.getState());
                            telemetry.addData("shooter status", robot.getShootState());
                            robot.launch(true, telemetry);
                        }
                        while (robot.getShootState() != Robot.ShootState.STOPPED) {
                            //Finish shooting without starting another iteration
                            telemetry.addData("auto status", "shooting ball " + i);
                            telemetry.addData("bot status", robot.getState());
                            telemetry.addData("shooter status", robot.getShootState());
                            robot.launch(false, telemetry);
                        }
                    }

                    stage = State.DRIVE_UP;
                    robot.cancel();
//                    if(System.currentTimeMillis()-lastStageTime>5000)
//                    {
//                        lastStageTime=System.currentTimeMillis();
//                        stage= DriveShootRedAuto.State.DONE; //Set to ONWARD if you want to do experimental stuff
//                        robot.cancel();
//                    }
                    break;

                case DRIVE_UP:
                    robot.timeDrive(0.5, 0.5, 500);

                    stage = State.TURN_TO_BEACON;
                    robot.cancel();

                case TURN_TO_BEACON:
                {

                    long turnTime = ( communism ? 500: 1500);
                    robot.timeDrive(-0.5, 0.5, turnTime);
                    stage = State.DRIVE_TO_BEACON;
                    robot.cancel();


                }

                case DRIVE_TO_BEACON:
                {
                    robot.timeDrive(0.7, 0.7, 1500);

                    stage = State.ALIGN_BEACONS;
                    robot.cancel();
                }

                case ALIGN_BEACONS:
                {
                    double turnAngle = (communism ? 0.5: -0.5);
                    robot.timeDrive( turnAngle, -turnAngle, 1000);
                    stage = State.WALL_FOLLOW;
                    robot.cancel();
                }

                case WALL_FOLLOW:
                {
                    while
                }

                case DONE:

                    break;

            }
        }
    }




    /**
     * Updates a telemetry Item's Message with the desired Caption
     *
     * @param Item  the Telemtry.Item to be updated
     * @param value The Updates message
     * @return the Updated Telemetry Item
     * note: store the Item and use it as the next updateTele parameter
     */

    public Telemetry.Item updateTele(Telemetry.Item Item, Object value) {
        String Caption = Item.getCaption();
        telemetry.removeItem(Item);
        return telemetry.addData(Caption, value);
    }
}
