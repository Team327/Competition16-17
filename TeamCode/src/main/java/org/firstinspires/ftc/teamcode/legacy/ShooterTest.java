package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by gssmrobotics on 12/2/2016.
 */
@Disabled
@Autonomous(name = "ShooterTest")

public class ShooterTest extends OpMode{

    DcMotor shooterMotor;
//    DcMotor motor2 = null;


    @Override
    public void init() {
        shooterMotor = hardwareMap.dcMotor.get("shooterMotor");
//        motor2 = hardwareMap.dcMotor.get("motor2");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setPower(.3);
//        motor2.setPower(1);
    }

    @Override
    public void loop()
    {

    }
}
