package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.competition.Robot;

/**
 * Created by gssmrobotics on 1/14/2017.
 */

@Disabled
@Autonomous(name = "JustShoot")
public class JustShootAuto extends OpMode {
    Robot robot;

    double startTime = -1;

    @Override
    public void init() {

        robot = new Robot(hardwareMap);

    }

    @Override
    public void loop() {
        if (startTime == -1) {
            startTime = System.currentTimeMillis();
        } else {
            if (System.currentTimeMillis() - startTime < 2000) {
                telemetry.addData("Status", System.currentTimeMillis() + ": SHOOTING");
                robot.shoot();
            } else {
                robot.stopShooter();
            }
        }
    }
}
