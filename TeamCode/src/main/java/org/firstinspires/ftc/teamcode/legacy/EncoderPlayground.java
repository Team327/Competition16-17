package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by gssmrobotics on 1/15/2017.
 */
@Disabled
public class EncoderPlayground extends OpMode {
    DcMotor motor = null;

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor1");
    }

    @Override
    public void loop() {
        motor.setPower(gamepad1.right_stick_y);
        telemetry.addData("Encoder Ticks", motor.getCurrentPosition());
    }
}
