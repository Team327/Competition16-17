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
        bot = new Robot(hardwareMap);
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
