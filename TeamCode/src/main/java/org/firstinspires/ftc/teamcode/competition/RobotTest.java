package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.competition.Robot;

/**
 * Created by gssmrobotics on 1/14/2017.
 */

@Autonomous(name = "Robot Test", group="Experimental")
public class RobotTest extends OpMode {
    Robot robot;
    RangeSensor leftFrontDist, leftRearDist, frontDist;

    @Override
    public void init() {
//        robot = new Robot(hardwareMap);

        leftFrontDist = new RangeSensor(hardwareMap.i2cDevice.get("leftFrontDist"), 0x12);
        leftRearDist = new RangeSensor(hardwareMap.i2cDevice.get("leftRearDist"), 0x16);
        frontDist = new RangeSensor(hardwareMap.i2cDevice.get("frontDist"), 0x14);
    }

    @Override
    public void loop() {
//        robot.launch(true, telemetry);
//        robot.setLeftPower(1);
//        robot.setRightPower(1);
//        telemetry.addData("Data", "Shooter:" + (robot.checkShooterEncoder() ? "Good" : (robot.shooterIsBusy() ? "Busy" : "Bad")));
//        telemetry.addData("Data", "Left:" + (robot.checkLeftEncoder() ? "Good" : "Bad"));
//        telemetry.addData("Data", "Right:" + (robot.checkRightEncoder() ? "Good" : "Bad"));

        telemetry.addData("leftFrontDist", leftFrontDist.getDistance(DistanceUnit.CM));
        telemetry.addData("leftRearDist", leftRearDist.getDistance(DistanceUnit.CM));
        telemetry.addData("frontDist", frontDist.getDistance(DistanceUnit.CM));
    }
}
