package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;


@Autonomous(name="MotorL1Test", group="Test")
public class MotorL1Test extends MyOpMode {
    public void runOpMode()
    {
        motorL1 = hardwareMap.dcMotor.get("motorL1");
        motorL1.setPower(1.0);
    }
}
