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
@Autonomous(name="Auto Red Range", group="Autonomous")
public class AutoRedRange extends MyAutonomous {
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
        while(!opModeIsActive())
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
        if(batteryLevel > 13.8)
            runSpinner(0.8);
        else
            runSpinner(0.86);
        pause(0.1);
        //moveAlongWallToBeacon(.3, 2.0, true);
        moveWithEncoders(-.4, 3075);
        pause(0.1);
        openServoDropper();
        pause(1.5);
        closeServoDropper();
        runSpinner(0.0);
        pause(0.1);
        moveWithEncoders(-0.4,3200);

        //moveWithEncoders(.5, 1000);
        gyroArcTurnRight(-0.3, yawDiff - 6.0);

        pause(0.25);
        turnParallelToWall(0.17);
        //17 max
        //9 min
        stabilizeAlongWallWithRangeForEncoderDist(-0.18, 1.0, 3.0, 13, false, 1000);
        stabilizeAlongWallWithRangeToBeacon(-0.125, 1.0, 3.0, 13, false);
        pause(0.1);
        turnParallelToWall(0.165);
        pause(0.1);
        driveAlongWallToBeacon(0.105, false);
        pause(0.1);

        pushButton();

        pause(0.25);
        if(colorB.getColor().equals("Blue"))
        {
            pause(5.0);
            pushButton();
        }

        turnParallelToWall(0.19);
        pause(0.1);

        moveWithEncoders(0.23, 1000);

        stabilizeAlongWallWithRangeForEncoderDist(0.18,1.0,3.0,13,false,700);
        pause(0.1);
        turnParallelToWall(0.19);
        pause(0.1);
        stabilizeAlongWallWithRangeToBeacon(0.125, 1.0, 3.0, 13, false);


        initCurtime();

        if(getCurTime() - startTime < 24.0) {
            pause(0.25);
            turnParallelToWall(0.18);
            pause(0.25);
            driveAlongWallToBeacon(-0.1, false);
            pushButton();
        }
        else if(getCurTime() - startTime < 26.0) {
            pause(0.25);
            turnParallelToWall(0.18);
            pause(0.25);
            pushButton();
        }
        else
        {
            pushButton();
        }

        //stabilizeAlongWallWithRangeToBeacon(-0.25,1.0,2.0,18,false);
        //moveAlongWallForUnits(0.6,1.0,2.0,20,true,3000);

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
