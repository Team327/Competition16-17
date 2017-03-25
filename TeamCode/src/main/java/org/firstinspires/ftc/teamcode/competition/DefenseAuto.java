package org.firstinspires.ftc.teamcode.competition;

import android.provider.Settings;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opengl.shaders.CubeMeshFragmentShader;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by gssmrobotics on 3/23/2017.
 */
@Autonomous(name = "Defensive Auto")
public class DefenseAuto extends OpMode {
    Robot robot;

    private final boolean communism   =     true    ;      //Are we the red alliance
    private final int     balls2shoot =     2       ;      //Number of balls to shoot
    private final long    delay       =     0       ;      //time in delay

    //telemetry
    private Telemetry.Item status;

    public enum State {
        DELAY,
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
        stage = State.DELAY;
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

        }
        if(lastStageTime==0) {
            lastStageTime = System.currentTimeMillis();
        } else {
            switch (stage) {
                case DELAY:
                {
                    if(System.currentTimeMillis() + delay > lastStageTime )
                    {
                        stage = State.DRIVE;
                    }
                    break;
                }
                case DRIVE:
                    robot.timeDrive(0.5, 0.5, 1450);
                    if(!robot.isBusy())
                    {
                        lastStageTime=System.currentTimeMillis();
                        stage= State.SHORT_WAIT;
                        robot.cancel();
                    }
                    break;
                case SHORT_WAIT:
                    if(System.currentTimeMillis() > lastStageTime + 500) {
                        lastStageTime = System.currentTimeMillis();
                        stage = State.SHOOT;
                        robot.cancel();
                    }
                    break;
                case SHOOT:
                    robot.launch(true,telemetry);

//                    for (int i = 1; i <= balls2shoot; i++) {
//                        while (robot.getShootState() != Robot.ShootState.COCKED_AND_LOADED) {
//                            //Start shooting a ball (get to COCKED_AND_READY state)
//                            telemetry.addData("auto status", "shooting ball " + i);
//                            telemetry.addData("bot status", robot.getState());
//                            telemetry.addData("shooter status", robot.getShootState());
//                            robot.launch(true, telemetry);
//                        }
//                        while (robot.getShootState() != Robot.ShootState.STOPPED) {
//                            //Finish shooting without starting another iteration
//                            telemetry.addData("auto status", "shooting ball " + i);
//                            telemetry.addData("bot status", robot.getState());
//                            telemetry.addData("shooter status", robot.getShootState());
//                            robot.launch(false, telemetry);
//                        }
//                    }
//
//                    stage = State.DRIVE_UP;
//                    robot.cancel();
                    if(System.currentTimeMillis()-lastStageTime>5000)
                    {
                        lastStageTime=System.currentTimeMillis();
                        stage= State.DRIVE_UP; //Set to ONWARD if you want to do experimental stuff
                        robot.cancel();
                    }
                    break;

                case DRIVE_UP:


                    robot.timeDrive(0.5, 0.5, 700);
                    if (!robot.isBusy()) {
                        stage = State.TURN_TO_BEACON;
                        robot.cancel();
                    }
                    break;
                case TURN_TO_BEACON:
                {

                    robot.setForward();
                    long turnTime = (communism ? 1400 : 500);
                    robot.timeDrive(-1, 1, turnTime);
                    if (!robot.isBusy()) {
                        stage = State.DRIVE_TO_BEACON;
                        robot.cancel();
                    }
                    break;
                }

                case DRIVE_TO_BEACON:
                {

                    robot.timeDrive(0.7, 0.7, 1500);
                    if (!robot.isBusy()) {
                        stage = State.ALIGN_BEACONS;
                        robot.cancel();
                    }
                    break;
                }

                case ALIGN_BEACONS:
                {

                    robot.timeDrive( 0.5, -0.5, 1000);
                    if (!robot.isBusy()) {
                        stage = State.WALL_FOLLOW;
                        robot.cancel();
                    }
                    break;
                }

                case WALL_FOLLOW:
                {
                    long wfTime = System.currentTimeMillis();
                    int direction = (communism ? 1 : -1);
                    if (System.currentTimeMillis() < time + 2800)
                    {
                        if(System.currentTimeMillis() < wfTime  + 1000)
                            robot.wallFollow(TeamConstants.WF_Kp, TeamConstants.WF_Kd, TeamConstants.WF_Ki,
                                            0.75 * direction, 75, telemetry );
                        else
                        {
                            direction *= -1;
                            wfTime = System.currentTimeMillis();
                        }

                    }
                    break;

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
