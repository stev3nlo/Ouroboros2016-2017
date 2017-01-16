package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

/**
 * Created by Morgannanez on 1/6/17.
 */

@Autonomous(name="Partner Red Shoot", group="Auto")
public class ShootRed extends MyAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        pause(10);
        initCurtime();
        initTime = getCurTime();
        moveWithEncoders(0.3, 4900,.93,1);
        runSpinner(1.0);
        pause(3);
        openServoDropper();
        pause(4);
        initTime = getCurTime();
        closeServoDropper();
    }
}
