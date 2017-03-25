package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import static org.firstinspires.ftc.teamcode.competition.VisionRobot.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.competition.VisionRobot.Alliance.RED;

/**
 * Created by roboticsteam on 3/8/2017.
 */
@Autonomous(name="Vision Auto")
public class VisionAuto extends LinearVisionOpMode{


    //creates a robot to use
    VisionRobot visRobot;

    //Stages of Autonomoose v2.1

    static final double Kp = 1;
    static final double Kd = -1;

    //single iteration variable
    private final boolean communism = true; //Are we the red alliance
    private final int balls2shoot = 2; //Number of balls to shoot

    public void initialize()
    {
        telemetry.addData("Status", "Initializing");telemetry.update();

        visRobot = new VisionRobot(hardwareMap, this); //for SimBot use new SimBot(hardwareMap, this, gamepad1);
        telemetry.addData("Status", "instantiated vis Robot"); telemetry.update();

        ///VisionOpMode initialization
        //enable camera extensions
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control
        telemetry.addData("Status","Enabled Extensions");telemetry.update();
        visRobot.init();
        telemetry.addData("Status","visRobot inited");telemetry.update();

        visRobot.setBackward(); //TODO remove this...
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //init the robot
        this.initialize();
        waitForVisionStart(); //TODO is this necessary
        waitForStart();
        telemetry.addData("Status","Passed wait for vision start"); telemetry.update();

        boolean x = true; //TODO remove

        //drive forward 4 feet
        //visRobot.distDriveTicks(1, 1, visRobot.dist2ticks(72));
        visRobot.distDriveTicks(1, 1, visRobot.dist2ticks(36)); //TODO remove
        telemetry.addData("Status", "Passed dist 2 ticks");
        telemetry.update();
        while (visRobot.isBusy()) {
            telemetry.addData("auto status", "drive past ball");
            visRobot.continueAction();
            //*************visRobot.safetyController(); //TODO remove
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done drive past ball");

        if (x) return;

        //turn 90 degrees (roughly 2 feet?)
        visRobot.distDriveTicks(-1, 1, visRobot.dist2ticks(24));
        while (visRobot.isBusy()) {
            telemetry.addData("auto status", "turn to beacon");
            visRobot.continueAction();
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done turn to beacon");

        //turn clockwise until beacon seen
        visRobot.detectBeacon(0.7, -0.4, 2000);
        while (visRobot.isBusy()) {
            telemetry.addData("auto status", "search for beacon 1");
            visRobot.continueAction();
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done search for beacon 1");

        //PD to beacon (max 5 seconds)
        visRobot.PDtoBeacon(Kp, Kd, 5000);
        while (visRobot.isBusy()) {
            telemetry.addData("auto status", "pd to beacon 1");
            visRobot.continueAction();
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done pd to beacon 1");

        int hitCount = 0; //counts hits
        do {
            //backup from beacon
            visRobot.drive2dist(0.8, 0.8, visRobot.hitDist, 2000, true);
            while (visRobot.isBusy()) {
                telemetry.addData("auto status", "hit beacon 1");
                visRobot.continueAction();
            }
            telemetry.addData("visbot status", visRobot.getState());
            telemetry.addData("auto status", "done hit beacon 1");

            //backup from beacon
            visRobot.drive2dist(-0.8, -0.8, visRobot.hitDist, 2000, false);
            while (visRobot.isBusy()) {
                telemetry.addData("auto status", "backup beacon 1");
                visRobot.continueAction();
            }
            telemetry.addData("visbot status", visRobot.getState());
            telemetry.addData("auto status", "done backup beacon 1");

            hitCount++;
        } while (!visRobot.needToClick(communism ? RED : BLUE) && hitCount <= 3);

        //turn a bit to line up with basket\
        visRobot.distDriveTicks(-0.5, 0.5, visRobot.angle2ticks(30));
        while (visRobot.isBusy()) {
            telemetry.addData("auto status", "turn to basket");
            visRobot.continueAction();
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done turn to basket");

        //backup to basket
        visRobot.distDriveTicks(-0.5, -0.5, visRobot.dist2ticks(24));
        while (visRobot.isBusy()) {
            telemetry.addData("auto status", "backup to basket");
            visRobot.continueAction();
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done backup to basket");

        //shoot
        for (int i = 1; i <= balls2shoot; i++) {
            while (visRobot.getShootState() != Robot.ShootState.COCKED_AND_LOADED) {
                //Start shooting a ball (get to COCKED_AND_READY state)
                telemetry.addData("auto status", "shooting ball " + i);
                telemetry.addData("visbot status", visRobot.getState());
                telemetry.addData("shooter status", visRobot.getShootState());
                telemetry.addData("auto status", "done pd to beacon 1");
                visRobot.launch(true, telemetry);
            }
            while (visRobot.getShootState() != Robot.ShootState.STOPPED) {
                //Finish shooting without starting another iteration
                telemetry.addData("auto status", "shooting ball " + i);
                telemetry.addData("visbot status", visRobot.getState());
                telemetry.addData("shooter status", visRobot.getShootState());
                telemetry.addData("auto status", "done pd to beacon 1");
                visRobot.launch(false, telemetry);
            }
        }

        //turn to far wall
        visRobot.distDriveTicks(0.5, -0.5, visRobot.angle2ticks(90));
        while (visRobot.isBusy()) {
            telemetry.addData("auto status", "turn far wall");
            visRobot.continueAction();
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done turn far wall");

        //drive to far wall
        visRobot.drive2dist(0.8, 0.8, 24, 6000, true);
        while (visRobot.isBusy()) {
            telemetry.addData("auto status", "drive far wall");
            visRobot.continueAction();
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done drive far wall");

        //turn to beacon 2
        visRobot.distDriveTicks(-0.7, 0.7, visRobot.angle2ticks(90));
        while (visRobot.isBusy()) {
            telemetry.addData("auto status", "turn beacon 2");
            visRobot.continueAction();
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done turn beacon 2");

        //pd to beacon 2
        visRobot.PDtoBeacon(Kp, Kd, 5000);
        while (visRobot.isBusy()) {
            telemetry.addData("auto status", "pd to beacon 2");
            visRobot.continueAction();
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done pd to beacon 2");

        hitCount = 0;
        do {
            //backup from beacon
            visRobot.drive2dist(0.8, 0.8, visRobot.hitDist, 2000, true);
            while (visRobot.isBusy()) {
                telemetry.addData("auto status", "hit beacon 2");
                visRobot.continueAction();
            }
            telemetry.addData("visbot status", visRobot.getState());
            telemetry.addData("auto status", "done hit beacon 2");

            //backup from beacon
            visRobot.drive2dist(-0.8, -0.8, visRobot.hitDist, 2000, false);
            while (visRobot.isBusy()) {
                telemetry.addData("auto status", "backup beacon 2");
                visRobot.continueAction();
            }
            telemetry.addData("visbot status", visRobot.getState());
            telemetry.addData("auto status", "done backup beacon 2");

            hitCount++;
        } while (!visRobot.needToClick(communism ? RED : BLUE) && hitCount <= 3);
    }
}
