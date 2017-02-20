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
@Autonomous(name="Auto Red Rollers", group="Autonomous")
public class AutoRedRollers extends MyAutonomous {
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
        runSpinner(curPowerOfMotorSpinner);
        pause(0.1);
        moveWithEncodersCoast(-.35, 1450, 1.0, 1);
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

        moveWithEncodersCoast(-0.22, 1550, 1.0, 1);

        telemetry.addData("arc turn", "Align With Wall");
        gyroArcTurnRight(-0.26, yawDiff - 4);

        telemetry.addData("move forward past 2nd beacon", "");
        moveWithEncodersCoastToWallOrToBeaconWithMinDist(-0.3, 4400, 1500,false,1.0, 0.8);
        pause(0.5);

        telemetry.addData("move backwards to 2nd beacon", "");
        driveAlongWallToBeacon(.15, false,1.0, 0.8);

        pushButtonWithRollers();
        pause(0.5);

        telemetry.addData("driving to beacon", "");
        driveToNextBeacon(0.24,false,1600,1.0,0.8);

        pushButtonWithRollers();

        pause(0.5);
        moveRollersUp();

        pause(rollerMovementTimeUp);
        holdRollersUp();
        double baseAngle = gyro.getYaw();
        double curAngle = baseAngle;
        while(opModeIsActive() && getAngleDiff(baseAngle,curAngle)<120)
        {
            moveBackwards(0.05,0.4);
            curAngle = gyro.getYaw();
            idle();
        }
    }
}
