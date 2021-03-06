package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

import java.io.IOException;

@Disabled//(name="PushButtonWithSpeedUntilFlashingTest", group="Test")
public class PushButtonWithSpeedStop extends MyAutonomous {

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        waitForStart();
        pushButtonWithSpeedUntilFlashing();
    }
}
