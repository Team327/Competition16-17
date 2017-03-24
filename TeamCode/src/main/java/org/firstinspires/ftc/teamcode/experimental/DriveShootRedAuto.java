package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.lasarobotics.vision.opmode.VisionOpMode;

/**
 * Created by gssmrobotics on 1/14/2017.
 */
@Autonomous(name = "DriveShootRedAuto")
public class DriveShootRedAuto extends VisionOpMode {
    VisionRobot robot;

    public enum State {DRIVE,SHOOT,BUMP, DONE}
    State stage;

    double lastStageTime = 0;

    @Override
    public void init() {
        robot = new VisionRobot(hardwareMap,this);
        stage = State.DRIVE;
        lastStageTime=0;

        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control
        robot.init();

        robot.reverseFront();
    }

    @Override
    public void loop() {
        if(lastStageTime==0) {
            lastStageTime = System.currentTimeMillis();
        }else {
            switch (stage) {
                case DRIVE:
                    robot.drive2dist(-.5, -.5, 6, 15000, false);
                    if(System.currentTimeMillis()-lastStageTime>15000)
                    {
                        lastStageTime=System.currentTimeMillis();
                        stage= State.SHOOT;
                        robot.brake();
                    }
                    break;
                case SHOOT:
                    robot.launch(true,telemetry);
                    if(System.currentTimeMillis()-lastStageTime>2000)
                    {
                        lastStageTime=System.currentTimeMillis();
                        stage= State.BUMP;
                        robot.launch(false,telemetry);
                    }
                    break;
                case BUMP:
                    robot.drive2dist(-.5, -.5, 1, 1000, false);
                    if(System.currentTimeMillis()-lastStageTime>1000)
                    {
                        lastStageTime=System.currentTimeMillis();
                        stage= State.DONE;
                        robot.brake();
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
