package org.firstinspires.ftc.teamcode.Autonomouses;

/**
 * Created by Morgannanez on 10/10/16.
 */

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RPMStabilizer;

public class AutoOpBlue extends MyOpMode {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        //moves towards white line in front of beacon
        //moveToWhiteLine(1.0);
        //turnRightToWhiteLine(1.0);

        // beacon presser method

        //turns robot to shooting position
        //gyroTurnLeft(1.0, 90.0);
        //gyroTurnLeftCorrection(1.0,90.0);

        //runs spinner
        initCurtime();
        double timeAtSpinnerStart = getCurTime();
        while(getCurTime()<timeAtSpinnerStart+1.0)
        {
            initCurtime();
            //shoot();
            idle();
        }

        //drops preplaced balls into spinner
        openServoDropper();
        initCurtime();
        double timeAtBallDrop = getCurTime();
        while(getCurTime()<timeAtBallDrop+5.0)
        {
            initCurtime();
            //shoot();
            idle();
        }
        closeServoDropper();

        //turns off spinnner
        runSpinner(0.0);
    }
}