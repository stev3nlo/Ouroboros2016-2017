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
            //telemetry.addData("yawDiff",yawDiff);
            boolean isGyroGood = yawDiff < 60.0;
            telemetry.addData("isGyroGood",isGyroGood);
            telemetry.update();
            idle();
        }
        waitForStart();
        initCurtime();
        double startTime = getCurTime();
        double batteryLevel = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        if(batteryLevel > 13.0)
            runSpinner(0.83);
        else
            runSpinner(0.88);
        pause(0.1);
        moveWithEncodersCoast(-.35, 1280, 1.0, 1);
        pause(0.1);
        openServoDropper();
        pause(1.5);
        closeServoDropper();
        runSpinner(0.0);
        pause(1.0);
        moveWithEncodersCoast(-0.22, 2170, 1.0, 1);


        //moveWithEncoders(.5, 1000);
        gyroArcTurnRight(-0.26, yawDiff -3);

        moveWithEncodersCoast(0.22, 3500, 1.0, 0.7);

        driveAlongWallToBeacon(-.15, false,1.0, 0.7);

        pushButtonWithRollers();

        pause(0.5);

        driveToNextBeacon(-0.25,false,2000,1.0,0.7);
        pause(0.5);
        pushButtonWithRollers();
    }
}
