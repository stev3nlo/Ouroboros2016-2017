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
@Autonomous(name="MoveAlongWallToBeaconTest", group="Test")
public class MoveAlongWallToBeaconTest extends MyAutonomous {
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
        waitForStart();
        //moveForwards(0.5);
        /*
        initCurtime();
        arcTurnRight(0.5);
        double startTime = getCurTime();
        while(getCurTime() < startTime + 5.0)
        {
            initCurtime();
            idle();
        }
        initCurtime();
        arcTurnRight(-0.5);
        startTime = getCurTime();
        while(getCurTime() < startTime + 5.0)
        {
            initCurtime();
            idle();
        }*/
        initCurtime();
        double startTime = getCurTime();
        double curTime = startTime;
        while(curTime - startTime < 1.0) {
            initCurtime();
            curTime = getCurTime();
        }
        //stabilizeAlongWallWithRangeForEncoderDist(-0.14, 1.0, 3.0, 11, true, 2500);

        stabilizeAlongWallWithRangeToBeacon(0.115, 1.0, 3.0, 11, true);

        /*runSpinner(1.0);
        pause(0.5);
        //moveAlongWallToBeacon(.3, 2.0, true);
        moveWithEncoders(.5, 3500);
        runSpinner(1.0);
        pause(1.0);
        openServoDropper();
        runSpinner(1.0);
        pause(1.5);
        closeServoDropper();
        runSpinner(0.0);
        pause(1.0);
        moveWithEncoders(0.5,1000);

        //moveWithEncoders(.5, 1000);
        gyroArcTurnLeft(0.3,45.0);
        */

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

        //moveAlongWallToBeacon(0.3, 1.0,2.0, 18,true);
        /*
        telemetry.addData("Pressing","beacon");
        telemetry.update();
        pause(3.0);

        moveAlongWallForUnits(-0.6,1.0,2.0,20,false,1000);
        telemetry.addData("Moving to","beacon");
        telemetry.update();
        moveAlongWallToBeacon(-0.3, 1.0,2.0,20, true);
        telemetry.addData("Pressing","beacon");
        telemetry.update();
        pause(3.0);
        */

       /* //travel forward to prine shooting spot
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
