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

    public void simpleStabilizingLoop(double t) throws InterruptedException
    {
        if(opModeIsActive()) {
            initCurtime();
            double startOfStablilizing = getCurTime();
            //telemetry.addData(""+startOfStablilizing,startOfStablilizing);
            while (opModeIsActive() && getCurTime() - startOfStablilizing < t) {
                telemetry.addData("startOfStablilizing", startOfStablilizing);
                telemetry.addData("getCurTime", getCurTime());
                runRPMStabilizationAuto();
                telemetry.update();
                runSpinner(curPowerOfMotorSpinner);
                initCurtime();
                try {
                    idle();
                } catch (InterruptedException e) {
                }
            }
        }
    }
}
