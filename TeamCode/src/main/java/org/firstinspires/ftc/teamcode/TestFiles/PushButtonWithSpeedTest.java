package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

import java.io.IOException;

@Disabled//(name="PushButtonWithSpeedTest", group="Test")
public class PushButtonWithSpeedTest extends MyOpMode {

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        waitForStart();
        pushButtonWithSpeed();
    }
}
