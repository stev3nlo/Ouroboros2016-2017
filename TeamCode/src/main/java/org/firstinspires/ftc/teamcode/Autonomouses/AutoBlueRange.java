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
        while(!opModeIsActive() && !isStopRequested())
        {
            yawDiff = getAngleDiff(startAngle,gyro.getYaw());
            telemetry.addData("OGangle",startAngle);
            telemetry.addData("newANGLE",gyro.getYaw());
            telemetry.addData("yawDiff",yawDiff);
            telemetry.update();
            idle();
        }
        waitForStart();
        //moveAlongWallToBeacon(.3, 2.0, true);
        initCurtime();
        double startTime = getCurTime();
        double batteryLevel = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        if(batteryLevel > 13.0)
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
        turnParallelToWallWithGyroSimple(0.195,0);
        //17 max
        //9 min
        //stabilizeAlongWallWithRangeForEncoderDist(0.14, 1.0, 3.0, 11, true, 1000);
        //stabilizeAlongWallWithRangeToBeacon(0.115, 1.0, 3.0, 11, true);
        pause(0.1);
        turnParallelToWallWithGyro(0.195, 0);
        pause(0.1);

        boolean foundBeacon = driveAlongWallToBeaconOrForUnits(0.105,true,400);
        if(!foundBeacon) {
            pause(0.2);
            driveAlongWallToBeacon(-0.105, true);
        }
        pause(0.1);
        pushButton();
        pause(0.25);

        if(colorB.getColor().equals("Red"))
        {
            pause(5.0);
            pushButton();
        }

        turnParallelToWallWithGyro(0.195, 0);

        //stabilizeAlongWallWithRangeForEncoderDist(-0.19, 1.0, 4.0, 11, true, 2000);
        //moveWithEncoders(-0.16,2000);
        pause(0.1);
        turnParallelToWallWithGyro(0.195, 0);
        pause(0.1);
        //stabilizeAlongWallWithRangeToBeacon(-0.115, 1.0, 3.0,10, true);


        initCurtime();

        if(getCurTime() - startTime < 20.0) {
            turnParallelToWallWithGyro(0.195, 0);
            pause(0.1);
            foundBeacon = driveAlongWallToBeaconOrForUnits(-0.105,true, 400);
            if(!foundBeacon) {
                pause(0.2);
                driveAlongWallToBeacon(0.105, true);
            }
            pause(0.1);
            turnParallelToWallWithGyro(0.195, 0);
            pause(0.1);
            pushButton();
        }
        else if(getCurTime() - startTime < 24.0) {
            turnParallelToWallWithGyro(0.195,0);
            pause(0.1);
            foundBeacon = driveAlongWallToBeaconOrForUnits(-0.105,true, 400);
            if(!foundBeacon) {
                pause(0.2);
                driveAlongWallToBeacon(0.105, true);
            }
            pushButton();
        }
        else if(getCurTime() - startTime < 28.0) {
            foundBeacon = driveAlongWallToBeaconOrForUnits(0.105,true, 400);
            pushButton();
        }
        else
        {
            pushButton();
        }

        /*
        double degreesToTurn = -26.0;
        telemetry.addData("degreesToTurn",degreesToTurn);
        //telemetry.addData("rangeBDist",rangeB.getDistance(DistanceUnit.CM));
        //telemetry.addData("rangeFDist",rangeF.getDistance(DistanceUnit.CM));
        telemetry.addData("turning", "");
        telemetry.update();
        gyroTurnRight(0.3, degreesToTurn);
        */

        /*
        moveAlongWallToBeacon(0.4, 1.0, 2.0, 16, true);
        pause(0.5);
        //moveAlongWallForUnits(-0.3, 1.0, 2.0, 14, true, 200);
        //pause(0.5);
        pushButton();
        pause(1.0);

        moveAlongWallForUnits(-0.4, 1.0, 2.0, 16, true, 1000);
        telemetry.addData("Moving to", "beacon");
        telemetry.update();

        moveAlongWallToBeacon(-0.4, 1.0, 2.0, 16, true);
        pause(1.0);
        //moveAlongWallForUnits(0.3,1.0,2.0,14,true,200);
        pushButton();


       /* //travel forward to prime shooting spot
        moveWithEncoders(-1, 550); //needs to be tested

        //shoot sequence
        //runs spinner
        initCurtime();
        double timeAtSpinnerStart = getCurTime();
        while(getCurTime()<timeAtSpinnerStart+1.0)
        {
            initCurtime();
            runSpinner(1.0);
            idle();
        }

        //drops preplaced balls into spinner
        openServoDropper();
        initCurtime();
        double timeAtBallDrop = getCurTime();
        while(getCurTime()<timeAtBallDrop+2.0)
        {
            initCurtime();
            runSpinner(1.0);
            idle();
        }
        closeServoDropper();

        //turns off spinnner
        numCyclesOfSlowingSpinner = 10;
        while(opModeIsActive() && numCyclesOfSlowingSpinner >= 0) {
            initCurtime();
            if (numCyclesOfSlowingSpinner >= 0 && getCurTime() - timeAtLastSpinnerSlowdown >= 0.2) {
                runSpinner(curPowerOfMotorSpinner * ((double) numCyclesOfSlowingSpinner / 10.0));
                timeAtLastSpinnerSlowdown = getCurTime();
                if (numCyclesOfSlowingSpinner > 0)
                    numCyclesOfSlowingSpinner--;
            }
        }

        //travel across the field to far beacon
        //using range1, stop x distance away
        while(rangeF.getUltraSonicDistance() > 10) //random number!! needs to be replaced
        {
          moveForwards(1.0);
        }
        stopMotors();

        //sets servos parallel to Wall
        //setServosParallel();

        if(rangeF.getUltraSonicDistance() > rangeB.getUltraSonicDistance())
            move(0.0, 0.3);
        //turn until both range1 and range2 are equal
        while(!(rangeF.getUltraSonicDistance() == rangeB.getUltraSonicDistance()))
        {
            move(0.0, 1.0);
        }
        stopMotors();


        //far beacon
        //move back until colorsensor senses right color
        //moveBackToBeacon("Blue");
        //press far beacon
        pushButton();

        //close beacon
        //move back until colorsensor senses right color
        //moveBackToBeacon("Blue");
        //press far beacon
        pushButton();
        

    }
    */
    }
}
