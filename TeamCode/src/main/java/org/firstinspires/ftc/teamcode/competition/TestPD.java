package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by gssmrobotics on 3/23/2017.
 */

@Autonomous(name="Test Wall Follow", group="Experimental")
public class TestPD extends OpMode {
    private Robot bot;
    private long startTime=-1;;
    private boolean forward=true;

    @Override
    public void init() {
        bot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        if(startTime == -1) {
            startTime = System.currentTimeMillis();
        }

        boolean prevForward = forward;
        forward = (System.currentTimeMillis() / 2000) % 2 == 0;
        //drive forward on every even multiple of 2 seconds (i.e. goes back and forth 2 seconds each)

        if(prevForward != forward) {
            bot.resetWallFollow();
        }

        forward = true; //TODO remove
        double power = forward ? 0.3 : -0.3;
        bot.wallFollow(TeamConstants.WF_Kp, TeamConstants.WF_Kd, TeamConstants.WF_Ki, power, 40, telemetry);

        telemetry.addData("front dist", bot.getLeftFrontDist());
        telemetry.addData("rear dist", bot.getLeftRearDist());
    }
}
