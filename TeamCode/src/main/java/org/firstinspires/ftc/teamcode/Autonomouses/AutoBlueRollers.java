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
@Autonomous(name="Auto Blue Rollers", group="Autonomous")
public class AutoBlueRollers extends MyAutonomous {
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
            boolean isGyroGood = yawDiff < 60.0;
            telemetry.addData("isGyroGood",isGyroGood);
            telemetry.update();
            idle();
        }
        waitForStart();
        initCurtime();
        double startTime = getCurTime();
        double batteryLevel = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        runSpinner(curPowerOfMotorSpinner + .03);
        pause(0.1);
        moveWithEncodersCoast(.3, 2750, 1.0, 1.0);
        pause(0.5);
        moveRollersDown();
        pause(rollerMovementTimeDown);
        stopRollers();
        pause(0.5);
        openServoDropper();
        pause(1.5);
        closeServoDropper();
        runSpinner(0.0);
        pause(1.0);


        telemetry.addData("move forwards to wall", "");
        moveWithEncodersCoast(0.28, 400, 1.0, 1);


        telemetry.addData("arc turn", "Align with wall");
        gyroArcTurnRight(0.24, yawDiff - 12);

        telemetry.addData("move forward past 2nd beacon", "");
        //moveWithEncodersCoastWithMaxTimeWithDriftAfterContact(0.36, 4300, 8.0, 1.0, 0.8, 6);
        moveWithEncodersCoastWithMaxTimeWithIncreasingDrift(0.36, 4600, 7.0, true, 1.0, 0.65);

        telemetry.addData("move backwards to 2nd beacon", "");
        driveAlongWallToBeacon(-0.165, true, 1.0, 0.7);
        pushButtonWithRollers();

        telemetry.addData("driving to beacon", "");
        driveToNextBeacon(-0.335,true,1500,1.0,0.7);

        pushButtonWithRollers();

        pause(0.5);
        moveRollersUp();

        pause(rollerMovementTimeUp);
        holdRollersUp();
        double baseAngle = gyro.getYaw();
        double curAngle = baseAngle;
        while(opModeIsActive() && getAngleDiff(baseAngle,curAngle)<120) //if runs into pole, doesn't stop; add second check statement with encoder change maybe
        {
            moveForwards(0.06,0.4);
            curAngle = gyro.getYaw();
            idle();
        }
        pause(0.5);
        moveWithEncodersCoast(0.4,450);
    }
}
