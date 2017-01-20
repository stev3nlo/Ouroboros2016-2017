package org.firstinspires.ftc.teamcode.Autonomouses;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRRange;


/**
 * Created by Steven on 11/14/2016.
 */
@Autonomous(name="Auto Red Simple", group="Autonomous")
public class AutoRedSimple extends MyAutonomous {
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
        moveWithEncodersCoast(-.35, 1300, 0.93, 1);
        pause(0.1);
        openServoDropper();
        pause(1.5);
        closeServoDropper();
        runSpinner(0.0);
        pause(1.0);
        moveWithEncodersCoast(-0.22, 2750, 0.93, 1);


        //moveWithEncoders(.5, 1000);
        gyroArcTurnRight(-0.2, yawDiff -14);

        pause(0.25);
        //turnParallelToWallWithGyro(0.195, 0);
        pause(0.1);
        //moveWithEncoders(0.2,-2000);
        boolean foundBeacon = driveAlongWallToBeaconOrForUnits(.105,false,2100, 0.93, 1);
        pause(.1);
        if (!foundBeacon)
        {
            driveAlongWallToBeacon(-.09, false,0.93, 1);
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

        if(opModeIsActive()) {
            pushButtonWithDistance();
            pause(1.5);
            moveBeaconPusherIn();
            pause(1.5);
        }

        pause(0.25);
        if(colorB.getColor().equals("Blue"))
        {
            pause(5.0);
            pushButton();
            pause(0.25);
        }
        //gyroTurnRight(0.15,getAngleDiff(prevGyroHeading,gyro.getYaw()-2.0));

        //turnParallelToWallWithGyro(0.195,0);
        pause(0.1);

        driveToNextBeacon(-0.22,false,2000,0.93,1.0);
        pause(0.5);
        pushButton();

        pause(0.25);
        if(colorB.getColor().equals("Blue"))
        {
            pause(5.0);
            pushButtonWithDistance();
        }
        moveWithEncodersCoast(-0.4,1500,0.93,1.0);
    }
}
