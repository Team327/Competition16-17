package org.firstinspires.ftc.teamcode.experimental;


/**
 * Current state or state of previous action
 */

public class States {
    private static final State[] busyStates = {State.PD_BEACON, State.TIME_DRIVE, State.DIST_DRIVE,
            State.DRIVE2DIST, State.DETECT_BEACON, State.HIT_BEACON, State.BACKUP}; //TODO add more

    public enum State {
        PD_BEACON, //Robot is in the middle of PD
        TIME_DRIVE, //Robot is in middle of timeDrive method
        DIST_DRIVE, //Robot is driving a set number of encoder ticks
        DRIVE2DIST, //Robot is in middle of drive2dist method
        DETECT_BEACON, //detectBeacon method for driving until beacon is seen
        HIT_BEACON, //hitBeacon method for hitting beacon after approaching it
        BACKUP, //backupFromBeacon method for backing up from beacon after hitting it
        SUCCESS, //Last action was successful
        FAILURE_TECH, //Failure for technical reasons (i.e. beacon navigation lost sight of beacon)
        FAILURE_TIMEOUT, //Robot timed out on previous task
        CANCELLED, //Previous action was cancelled
        NULL //What it starts out as -> means absolutely nothing
    }

    /**
     * Is the robot busy? (Descriptive commenting thanks to FIRST API)
     * @return Returns true if the robot is busy
     */
    public static boolean isBusy(State state) {
        return contains(busyStates, state);
    }

    public static boolean contains(State[] list, State item) {
        for(State i : list) {
            if(item == i) return true;
        }
        return false;
    }
}
