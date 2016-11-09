package org.firstinspires.ftc.teamcode;

/**
 * Created by Morgannanez on 10/6/16.
 */
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RPMStabilizer;

public class AutoOpRed extends MyOpMode {

/* NEED TO ADD BEACON PRESS METHOD*/
    
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        //moves towards white line in front of beacon
        moveToWhiteLine(1.0);
        turnRightToWhiteLine(1.0);

        //need beacon press method

        //turns robot to shooting position
        gyroTurnRight(1.0, 90.0);
        gyroTurnRightCorrection(1.0,90.0);

        //runs spinner
        initCurtime();
        double timeAtSpinnerStart = getCurTime();
        while(getCurTime()< timeAtSpinnerStart+1.0) {
            initCurtime();
            //shoot();
            idle();
        }

        //drops preplaced balls into spinner
        openServoDropper();
        initCurtime();
        double timeAtBallDrop = getCurTime();
        while(getCurTime()<timeAtBallDrop+5.0) {
            initCurtime();
            //shoot();
            idle();
        }
        closeServoDropper();

        //turns off spinner
        runSpinner(0.0);
    }
}
