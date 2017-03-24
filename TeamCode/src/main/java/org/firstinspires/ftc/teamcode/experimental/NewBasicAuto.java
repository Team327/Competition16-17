package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by gssmrobotics on 3/24/2017.
 */

@Disabled
@Autonomous(name="New Basic Auto")
public class NewBasicAuto extends LinearOpMode {
    Robot bot;
    long startTime=-1, prevTime=0;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Robot(hardwareMap);
        waitForStart();

        telemetry.addData("Status", "Thouroghly incomplete");
    }
}
