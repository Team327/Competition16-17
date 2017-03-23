package org.firstinspires.ftc.teamcode.experimental;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by gssmrobotics on 3/23/2017.
 */

public class DefenseAuto extends LinearVisionOpMode {
    VisionRobot VisBot;

    static final double Kp = 1;
    static final double Kd = -1;
    static final double Ki = 1;
    //telemetry
    private Telemetry.Item status;

    //Time
    private long time;

    //single iteration variable
    private final boolean communism = true; //Are we the red alliance
    private final int balls2shoot = 2; //Number of balls to shoot


    public void initialize() {

        status = updateTele(status, "Initializing...");

        //Enable Vision Robot
        VisBot = new VisionRobot(hardwareMap, this);
        status = updateTele(status, "Initializing Vision Robot");


        //Enable extensions
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control
        status = updateTele(status, "Initialized Extensions");

        VisBot.init();
        status = updateTele(status, "Initialized Vision Robot");


    }

    public void runOpMode() throws InterruptedException {
        status.addData("Status:", "Begun");
        this.initialize();

        status = updateTele(status, "Ready to go");
        waitForStart();
        status = updateTele(status, "Started");
        time = System.currentTimeMillis();

        int direction = (communism ? -1 : 1);
        VisBot.distDriveTicks(direction, direction, VisBot.dist2ticks(60));
        status = updateTele(status, "Driving");

        while (VisBot.isBusy()) {
            status = updateTele(status, "Driving to Center Vortex");
            VisBot.continueAction();

        }


        //copied from Vision Auto
        for (int i = 1; i <= balls2shoot; i++) {
            while (VisBot.getShootState() != Robot.ShootState.COCKED_AND_LOADED) {
                //Start shooting a ball (get to COCKED_AND_READY state)
                status = updateTele(status, "Shooting");
                VisBot.launch(true, telemetry);
            }
            while (VisBot.getShootState() != Robot.ShootState.STOPPED) {
                //Finish shooting without starting another iteration
                status = updateTele(status, "Shooting");
                VisBot.launch(false, telemetry);
            }
        }

        VisBot.distDriveTicks(0.5, -0.5, VisBot.angle2ticks(45));
        status = updateTele(status, "Turning Around Ball");
        while (VisBot.isBusy()) {
            VisBot.continueAction();
        }
        while (System.currentTimeMillis() < time + 10000) {
        }

        status = updateTele(status, "Running to block Beacon");
        VisBot.distDriveTicks(-.8, -.8, VisBot.dist2ticks(54));
        while (VisBot.isBusy()) {
            VisBot.continueAction();
        }

        status = updateTele(status, "Turning along beacons");
        VisBot.distDriveTicks(-0.5, 0.5, VisBot.angle2ticks(90));
        while (VisBot.isBusy()) {
            VisBot.continueAction();
        }

        time = System.currentTimeMillis();
        while (System.currentTimeMillis() < time + 1000) {
            VisBot.wallFollow(Kp, Kd, Ki, 1, telemetry);
        }
        VisBot.resetWallFollow();
        time = System.currentTimeMillis();
        while (System.currentTimeMillis() < time + 1000) {
            VisBot.wallFollow(Kp, Kd, Ki, -1, telemetry);
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
