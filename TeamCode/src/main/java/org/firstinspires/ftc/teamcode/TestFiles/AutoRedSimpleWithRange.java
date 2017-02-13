package org.firstinspires.ftc.teamcode.TestFiles;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRRange;


/**
 * Created by Steven on 11/14/2016.
 */
@Disabled//(name="Auto Red Simple with Range", group="Autonomous")
public class AutoRedSimpleWithRange extends MyAutonomous {
    //travel 3ft to preferred shooting place
    //shoot
    //travel across the field to far beacon
    //using range1, stop x distance away
    //turn until both range1 and range2 are equal
    //press far beacon
    //back up until at closest beacon using color sensor
    //press beacon


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
        moveWithEncodersCoast(-.35, 1500, 0.93, 1);
        pause(0.1);
        openServoDropper();
        pause(1.5);
        closeServoDropper();
        runSpinner(0.0);
        pause(0.1);
        moveWithEncodersCoast(-0.28, 1750, 0.93, 1);

        //moveToWallWithRange()
        //moveWithEncoders(.5, 1000);
        gyroArcTurnRight(-0.2, yawDiff -8);

        pause(0.25);
        //turnParallelToWallWithGyro(0.195, 0);
        pause(0.1);
        //moveWithEncoders(0.2,-2000);
        boolean foundBeacon = driveAlongWallToBeaconOrForUnits(.12,false,2500, 0.93, 1);
        pause(.1);
        if (!foundBeacon)
        {
            driveAlongWallToBeacon(-.12, false,0.93, 1);
        }
        telemetry.addData("USDF",rangeF.getUltraSonicDistance());
        telemetry.addData("USDB",rangeB.getUltraSonicDistance());
        telemetry.update();
        pause(2.0);

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

        pushButton();

        pause(0.25);
        if(colorB.getColor().equals("Blue"))
        {
            pause(5.0);
            pushButton();
            pause(0.25);
        }
        gyroTurnRight(0.15,getAngleDiff(prevGyroHeading,gyro.getYaw()-2.0));

        //turnParallelToWallWithGyro(0.195,0);
        pause(0.1);

        moveWithEncodersCoast(-0.36, 2000,.93,1);
        pause(0.25);

        driveAlongWallToBeacon(-.12,false,.93,1);
        pushButton();

        pause(0.25);
        if(colorB.getColor().equals("Blue"))
        {
            pause(5.0);
            pushButton();
        }
        moveWithEncodersCoast(-0.4,1500,0.93,1.0);
    }
}
