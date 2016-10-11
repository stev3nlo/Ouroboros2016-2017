package org.firstinspires.ftc.teamcode;

/**
 * Created by Morgannanez on 10/10/16.
 */

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RPMStabilizer;

public class AutoOpBlue extends MyOpMode {

    public void runOpMode() throws InterruptedException {
        moveToWhiteLine(1.0);
        turnRightToWhiteLine(1.0);

        initCurtime();
        double timeAtSpinnerStart = getCurTime();
        while(getCurTime()<timeAtSpinnerStart+1.0) {
            initCurtime();
            shoot();
            idle();
        }
        openServoDropper();
        initCurtime();
        double timeAtBallDrop = getCurTime();
        while(getCurTime()<timeAtBallDrop+5.0) {
            initCurtime();
            shoot();
            idle();
        }
        closeServoDropper();

        runSpinner(0.0);
    }
}