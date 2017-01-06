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
        initCurtime();
        initTime = getCurTime();
        simpleStabilizingLoop(1.0);
        moveWithEncoders(.8, 3700);
        simpleStabilizingLoop(1.0);
        openServoDropper();
        initTime = getCurTime();
        simpleStabilizingLoop(1.5);
        closeServoDropper();
    }
}
