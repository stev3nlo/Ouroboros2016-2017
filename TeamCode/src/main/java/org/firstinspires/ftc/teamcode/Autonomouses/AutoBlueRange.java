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
@Autonomous(name="Auto Blue Range", group="Autonomous")
public class AutoBlueRange extends MyAutonomous {
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
        while (!opModeIsActive() && !isStopRequested()) {
            yawDiff = getAngleDiff(startAngle, gyro.getYaw());
            telemetry.addData("OGangle", startAngle);
            telemetry.addData("newANGLE", gyro.getYaw());
            telemetry.addData("yawDiff", yawDiff);
            telemetry.update();
            idle();
        }
        waitForStart();
        //moveAlongWallToBeacon(.3, 2.0, true);
        initCurtime();
        double startTime = getCurTime();
        double batteryLevel = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        if (batteryLevel > 13.0)
            runSpinner(0.82);
        else
            runSpinner(0.92);
        pause(0.1);
        moveWithEncoders(0.32, 3600);
        pause(0.1);
        openServoDropper();
        pause(1.5);
        closeServoDropper();
        runSpinner(0.0);
        pause(0.25);
        moveWithEncoders(0.25, 3300);

        //moveWithEncoders(.5, 1000);
        gyroArcTurnRight(0.2, yawDiff - 13.0);
        pause(0.1);
        turnParallelToWallWithGyroSimple(0.195, 0);
        //17 max
        //9 min
        //stabilizeAlongWallWithRangeForEncoderDist(0.14, 1.0, 3.0, 11, true, 1000);
        //stabilizeAlongWallWithRangeToBeacon(0.115, 1.0, 3.0, 11, true);
        pause(0.1);
        turnParallelToWallWithGyro(0.195, 0);
        pause(0.1);

        boolean foundBeacon = driveAlongWallToBeaconOrForUnits(0.105, true, 400);
        if (!foundBeacon) {
            pause(0.2);
            driveAlongWallToBeacon(-0.105, true);
        }
        pause(0.1);
        pushButton();
        pause(0.25);

        if (colorB.getColor().equals("Red")) {
            pause(5.0);
            pushButton();
        }

        turnParallelToWallWithGyro(0.195, 0);

        //Stabilizes along the wall for encoder distance
        stabilizeAlongWall(-0.16, 4.0, 11, true, true, 2000);
        pause(0.1);
        turnParallelToWallWithGyro(0.195, 0);
        pause(0.1);
        stabilizeAlongWall(-0.115, 4.0,11, true, false,-1);


        initCurtime();

        if (getCurTime() - startTime < 20.0) {
            turnParallelToWallWithGyro(0.195, 0);
            pause(0.1);
            foundBeacon = driveAlongWallToBeaconOrForUnits(-0.105, true, 400);
            if (!foundBeacon) {
                pause(0.2);
                driveAlongWallToBeacon(0.105, true);
            }
            pause(0.1);
            turnParallelToWallWithGyro(0.195, 0);
            pause(0.1);
            pushButton();
        } else if (getCurTime() - startTime < 24.0) {
            turnParallelToWallWithGyro(0.195, 0);
            pause(0.1);
            foundBeacon = driveAlongWallToBeaconOrForUnits(-0.105, true, 400);
            if (!foundBeacon) {
                pause(0.2);
                driveAlongWallToBeacon(0.105, true);
            }
            pushButton();
        } else if (getCurTime() - startTime < 28.0) {
            foundBeacon = driveAlongWallToBeaconOrForUnits(0.105, true, 400);
            pushButton();
        } else {
            pushButton();
        }
    }
}
