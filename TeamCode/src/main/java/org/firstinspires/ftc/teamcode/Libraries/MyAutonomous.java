package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "MyAutonomous", group = "Test")
@Disabled
public class MyAutonomous extends MyOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initializeSensors();
    }
}
