package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

import java.io.IOException;

/**
 * Created by Morgannanez on 1/5/17.
 */

@Autonomous(name="PushButtonTest", group="Test")

public class PushButtonTest extends MyOpMode {

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        waitForStart();
        moveWithEncoders(0.2,6000);
    }
}
