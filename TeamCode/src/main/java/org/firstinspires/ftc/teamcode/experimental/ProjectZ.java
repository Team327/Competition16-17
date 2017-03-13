package org.firstinspires.ftc.teamcode.experimental;

import android.media.AudioManager;
import android.media.MediaPlayer;
import android.media.ToneGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;

/**
 * Created by roboticsteam on 3/10/2017.
 */

@TeleOp(name="ProjectZ")
public class ProjectZ extends OpMode {
    /**
     * New version (plays a song while looping)
     */
    private MediaPlayer mp;
    private final String mediaPath = "/storage/emulated/0/Music";
    private final String mediaFile = "projectZ.mp3";
    private boolean startedPlaying = false;

    @Override
    public void init() {
        mp = new MediaPlayer();

        if(mp.isPlaying()) {
            mp.stop();
        }

        try {
            mp.setDataSource(mediaPath + "/" + mediaFile);
            mp.prepare();
        } catch (IOException e) {
            telemetry.addData("\'Error" , "Oh no :(\' - 5893");
            telemetry.update();
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {
        if(!startedPlaying) {
            mp.start();
            startedPlaying = true;
        }
    }

    @Override
    public void stop() {
        mp.stop();
    }
}
