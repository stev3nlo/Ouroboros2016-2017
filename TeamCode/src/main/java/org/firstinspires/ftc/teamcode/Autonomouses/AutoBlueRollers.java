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
        double batteryLevel = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        runSpinner(1.0);
        pause(0.1);
        moveWithEncodersCoast(.3, 2100, 1.0, 1.0);
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
        moveWithEncodersCoast(0.28, 440, 1.0, 1);


        telemetry.addData("arc turn", "");
        gyroArcTurnRight(0.2, yawDiff - 12);

        telemetry.addData("move forward past 2nd beacon", "");
        moveWithEncodersCoast(0.22, 4000, 1.0, 0.8);

        telemetry.addData("move backwards to 2nd beacon", "");
        driveAlongWallToBeacon(-0.16, true, 1.0, 0.7);

        pushButtonWithRollers();
        pause(0.5);

		telemetry.addData("driving to beacon", "");
        driveToNextBeacon(-0.36,true,1500,1.0,0.75);
        pause(0.5);

        pushButtonWithRollers();

        pause(0.5);
        moveRollersUp();

        pause(rollerMovementTimeUp);
        holdRollersUp();
        double baseAngle = gyro.getYaw();
        double curAngle = baseAngle;
        while(opModeIsActive() && getAngleDiff(baseAngle,curAngle)<120)
        {
            moveForwards(0.08,0.4);
            curAngle = gyro.getYaw();
            idle();
        }
        pause(.5);
    }
}
