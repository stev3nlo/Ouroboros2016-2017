package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

import java.io.IOException;

/**
 * Created by Morgannanez on 1/5/17.
 */

@Autonomous(name="PushButtonTest", group="Test")

public class PushButtonTest extends MyAutonomous {

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        waitForStart();
        double distToPush = 0.0;
        boolean isServoOut = false;
        while(opModeIsActive())
        {
            if(gamepad1.x)
                distToPush -= 0.05;
            else if(gamepad1.b)
                distToPush += 0.05;
            if(distToPush < 0.0)
                distToPush = 0.0;
            else if(distToPush > 1.0)
                distToPush = 1.0;
            if(gamepad1.left_trigger > 0.6 || gamepad1.right_trigger > 0.6)
            {
                isServoOut = true;
            }
            else if(gamepad1.right_bumper || gamepad1.left_bumper)
                isServoOut = false;
            if(isServoOut)
            {
                servoBeaconPusher.setPosition(distToPush);
            }
            else
                servoBeaconPusher.setPosition(1.0);
            telemetry.addData("USDF",rangeF.getUltraSonicDistance());
            telemetry.addData("USDB",rangeB.getUltraSonicDistance());
            telemetry.addData("isServoOut",isServoOut);
            telemetry.addData("distToPush",distToPush);
            telemetry.update();
            pause(0.1);
            idle();
        }
        //moveWithEncoders(0.2,6000,.93,1);
    }
}