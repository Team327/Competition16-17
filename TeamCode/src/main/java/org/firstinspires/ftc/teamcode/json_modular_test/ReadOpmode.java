package org.firstinspires.ftc.teamcode.json_modular_test;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by roboticsteam on 4/5/2017.
 */

@Autonomous(name="FileReader", group="experimental")
public class ReadOpmode extends LinearOpMode {

    /* Checks if external storage is available to at least read */
    public boolean isExternalStorageReadable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state) ||
                Environment.MEDIA_MOUNTED_READ_ONLY.equals(state)) {
            return true;
        }
        return false;
    }

    /* Writes an example  */
    public boolean writeExample() {
        return false; //TODO return true if the write is successful
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
