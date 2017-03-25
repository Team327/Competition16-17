package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by gssmrobotics on 3/24/2017.
 */

@Autonomous(name="Uber Basic Auto")
public class UberBasicAuto extends LinearOpMode {
    DcMotor left, right, intake;
    long startTime = 0,
            driveTime = 1180;
    //Note for parking on corner vortex driveTime = //TODO

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        intake = hardwareMap.dcMotor.get("intake");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        startTime = System.currentTimeMillis();
        left.setPower(0.75);
        right.setPower(0.75);
        intake.setPower(-1);
        while (System.currentTimeMillis() < (startTime + driveTime)) {
            //drives forward for specified time
            telemetry.addData("Left Loc", left.getCurrentPosition());
            telemetry.addData("Right Loc", right.getCurrentPosition());
        }
        left.setPower(0);
        right.setPower(0);
        intake.setPower(0);
        telemetry.addData("Status", "Purposefully immobilized");
    }
}