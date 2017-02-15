package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by gssmrobotics on 11/2/2016.
 */

@Disabled
@TeleOp
public class BasicTeleOp extends OpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight,intake, shooter;

    @Override
    public void init() {
        intake = hardwareMap.dcMotor.get("intake");
        shooter = hardwareMap.dcMotor.get("shooter");
        frontLeft = hardwareMap.dcMotor.get("front-left");
        frontRight = hardwareMap.dcMotor.get("front-right");
        backLeft = hardwareMap.dcMotor.get("back-left");
        backRight = hardwareMap.dcMotor.get("back-right");
    }

    @Override
    public void loop() {

        //Pushbot left and right stick to tank drive
        double right = gamepad1.right_stick_y;
        double left = -gamepad1.left_stick_y;
        double intakeSpeed = gamepad1.left_trigger;
        frontRight.setPower(right);
        backRight.setPower(right);
        frontLeft.setPower(left);
        backLeft.setPower(left);
        intake.setPower(intakeSpeed);
    }
}
