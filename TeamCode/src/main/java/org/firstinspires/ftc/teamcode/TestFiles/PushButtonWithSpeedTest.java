package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

import java.io.IOException;

@Autonomous(name="PushButtonWithSpeedTest", group="Test")

public class PushButtonWithSpeedTest extends MyOpMode {

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        waitForStart();
        pushButtonWithSpeed();
    }
}
