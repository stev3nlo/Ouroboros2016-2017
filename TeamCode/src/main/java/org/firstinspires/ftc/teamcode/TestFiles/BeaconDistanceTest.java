package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;

/**
 * Created by Morgan on 11/1/2016.
 */
@Autonomous(name = "BeaconDIstanceTest", group = "Test")
public class BeaconDistanceTest extends MyAutonomous {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        while(!opModeIsActive() && !isStopRequested())
        {

            telemetry.addData("rangeF", rangeF.getUltraSonicDistance());
            telemetry.addData("rangeB", rangeB.getUltraSonicDistance());
            pushButtonWithDistanceOutput();
            telemetry.update();
            idle();
        }
        waitForStart();
        while (opModeIsActive()) {
            pushButtonWithDistance();
            idle();
        }
    }
}
