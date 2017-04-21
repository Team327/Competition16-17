package org.firstinspires.ftc.teamcode.annotations_test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by hdamron1594 on 4/21/17.
 */

@TeleOp(name="Annotations Test", group="experimental")
public class AnnotationTest extends OpMode {
    RobotCore core;

    @Override
    public void init() {
        Log.d("Status","Getting RobotCore");
        core = RobotCore.getInstance();
        Log.d("Status","Got RobotCore");
        //core.getModuleOrCreate("dependent");
    }

    @Override
    public void loop() {

    }
}
