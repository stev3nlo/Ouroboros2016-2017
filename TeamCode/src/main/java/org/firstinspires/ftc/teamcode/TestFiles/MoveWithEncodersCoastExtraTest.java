package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

/**
 * Created by spencersharp on 1/16/17.
 */
@Autonomous(name="MoveWithEncodersCoastExtraTest", group="Test")
public class MoveWithEncodersCoastExtraTest extends MyOpMode
{
    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        waitForStart();
        moveWithEncodersCoastExtra(0.35,2000,1.0,0.86);
    }
}
