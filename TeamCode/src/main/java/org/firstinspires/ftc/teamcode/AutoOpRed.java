package org.firstinspires.ftc.teamcode;

/**
 * Created by Morgannanez on 10/6/16.
 */
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RPMStabilizer;

public class AutoOpRed extends MyOpMode {


    public void runOpMode() {

        moveToWhiteLine(1.0);
        turnRightToWhiteLine(1.0);

        setServoDropperPosition(1.0);
        openServoDropper();
        closeServoDropper();

        //runs shooter

        // need loop for time
        initCurtime();
        initShooter();
        shoot();

        runSpinner(0.0);



    }
}
