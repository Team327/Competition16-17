package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by gssmrobotics on 1/14/2017.
 */

@Autonomous(name = "Robot Test")
public class RobotTest extends OpMode {
    Robot robot;


    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        robot.launch();
        robot.setLeftPower(1);
        robot.setRightPower(1);
        telemetry.addData("Data", "Shooter:" + (robot.checkShooterEncoder() ? "Good" : "Bad"));
        telemetry.addData("Data", "Left:" + (robot.checkLeftEncoder() ? "Good" : "Bad"));
        telemetry.addData("Data", "Right:" + (robot.checkRightEncoder() ? "Good" : "Bad"));
    }
}
