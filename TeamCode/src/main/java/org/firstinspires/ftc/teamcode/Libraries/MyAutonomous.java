package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "MyAutonomous", group = "Test")
@Disabled
public class MyAutonomous extends MyOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        /*motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
        motorSpinner.setMaxSpeed(4500);
        initializeSensors();
        pause(1.0);
        initAngle = gyro.getYaw();
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
                idle();
            }
        }
    }

    public double getDegreesToTurnFromDistances(double d1, double d2)
    {
        double degrees;
        if(d1>d2)
        {
            double a = d1;
            double b = d2;
            double c = 25.0;
            double radians = Math.atan(c/(a-b));
            degrees = Math.toDegrees(radians);
            degrees = Math.abs(degrees);
            degrees = 90-degrees;
            degrees *= -1;
        }
        else
        {
            double a = d2;
            double b = d1;
            double c = 25.0;
            double radians = Math.atan(c/(a-b));
            degrees = Math.toDegrees(radians);
            degrees = Math.abs(degrees);
            degrees = 90-degrees;
        }
        return degrees;
    }
}
