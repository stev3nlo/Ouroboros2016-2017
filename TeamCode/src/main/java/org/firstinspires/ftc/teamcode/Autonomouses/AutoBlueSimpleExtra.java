package org.firstinspires.ftc.teamcode.Autonomouses;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;
import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRRange;

/**
 * Created by Morgannanez on 1/18/17.
 */

@Autonomous(name="Auto Blue Simple Extra", group="Autonomous")
public class AutoBlueSimpleExtra extends MyAutonomous {

    public void runOpMode() throws InterruptedException {
        //rangeF.initializeSensors();
        //rangeB.initializeSensors();
        //initializeBlueServos();
        super.runOpMode();
        //initializeSensors();
        double startAngle = gyro.getYaw();
        double yawDiff = 0.0;
        while(!opModeIsActive() && !isStopRequested())
        {
            yawDiff = getAngleDiff(startAngle,gyro.getYaw());
            telemetry.addData("yawDiff",yawDiff);
            telemetry.update();
            idle();
        }
        waitForStart();
        initCurtime();
        double startTime = getCurTime();
        double batteryLevel = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        if(batteryLevel > 13.0)
            runSpinner(0.8);
        else
            runSpinner(0.88);
        pause(0.1);
        //moveAlongWallToBeacon(.3, 2.0, true);
        moveWithEncodersCoastExtra(0.35, 1500, 1.0, 0.86);
        pause(0.1);
        openServoDropper();
        pause(1.5);
        closeServoDropper();
        runSpinner(0.0);
        pause(0.1);
        moveWithEncodersCoastExtra(0.28, 1580, 1.0, 0.86);


        //moveWithEncoders(.5, 1000);
        gyroArcTurnLeft(0.2, yawDiff);

        pause(0.25);
        //turnParallelToWallWithGyro(0.195, 0);
        pause(0.1);
        //moveWithEncoders(0.2,-2000);
        boolean foundBeacon = driveAlongWallToBeaconOrForUnits(-.12,false,1400, 1.0,0.86);
        pause(.1);
        if (!foundBeacon)
        {
            driveAlongWallToBeacon(-12, false,0.93, 1);
        }

        pause(0.1);
        /*
        boolean foundBeacon = driveAlongWallToBeaconOrForUnits(-0.105,false,400);
        if(!foundBeacon) {
            pause(0.2);
            driveAlongWallToBeacon(0.105, false);
        }
        pause(0.1);
        */
        double prevGyroHeading = gyro.getYaw();

        pushButtonWithDistance();

        pause(0.25);
        if(colorB.getColor().equals("Red"))
        {
            pause(5.0);
            pushButtonWithDistance();
            pause(0.25);
        }

        telemetry.addData("USDF",rangeF.getUltraSonicDistance());
        telemetry.addData("USDB",rangeB.getUltraSonicDistance());
        telemetry.update();
        pause(1.0);
        double USDF = rangeF.getUltraSonicDistance();
        double USDB = rangeB.getUltraSonicDistance();
        telemetry.addData("USDF",USDF);
        telemetry.addData("USDB",USDB);
        double angleDiff = getAngleDiff(prevGyroHeading,gyro.getYaw());
        telemetry.addData("angleDiff",angleDiff);
        telemetry.update();
        if(Math.abs(USDF-USDB) > 2.0) {
            gyroTurnLeft(0.15, angleDiff);
        }
        //turnParallelToWallWithGyro(0.195,0);
        pause(0.1);

        moveWithEncodersCoast(0.36, 2000,.93,1);
        pause(0.25);

        driveAlongWallToBeacon(.12, false, .93, 1);
        pushButtonWithDistance();

        pause(0.25);
        if(colorB.getColor().equals("Red"))
        {
            pause(5.0);
            pushButtonWithDistance();
        }
        moveWithEncodersCoast(0.4,1500,0.93,1.0);
    }

}
