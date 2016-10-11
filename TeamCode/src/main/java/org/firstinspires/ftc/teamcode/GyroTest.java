package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorAdafruitIMU;

/**
 * Created by Morgannanez on 10/10/16.
 */

public class GyroTest extends MyOpMode {

    SensorAdafruitIMU gyroTest;

    public void initialize(){

        gyroTest = new SensorAdafruitIMU();
    }

    public void runOpMode(){
        telemetry.addData("Yaw", gyroTest.getYaw());
        telemetry.addData("Roll",gyroTest.getRoll());
        telemetry.addData("Pitch", gyroTest.getPitch());
    }
}