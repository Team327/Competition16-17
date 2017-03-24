package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by gssmrobotics on 3/22/2017.
 */

@Autonomous(name = "Just Shoot *", group="Experimental")
public class JustShootAsterisk extends OpMode {
    private Robot bot;

    private long prevTime = 0;
    private final long duration = 5000; //5 seconds delay

    @Override
    public void init() {
        try {
            bot = new Robot(hardwareMap);
        } catch(Exception e) {
            telemetry.addData("ERROR ERROR ERROR", "Unable to connect to robot");
            bot = new SimBot(hardwareMap, this, gamepad2);
            //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
            //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
            //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
            //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
            //TODO remove this - it's a super security concern
        }
        prevTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        boolean init = (System.currentTimeMillis() - prevTime) > duration;
        if (init) {
            prevTime = System.currentTimeMillis();
        }
        bot.launch(init, telemetry);
    }
}
