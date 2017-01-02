package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

/**
 * Created by spencersharp on 12/31/16.
 */
@Autonomous(name = "ArcTurnLeftTest", group = "Test")

public class ArcTurnLeftTest extends MyOpMode{
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            arcTurnLeft(0.2);
        }
    }
}
