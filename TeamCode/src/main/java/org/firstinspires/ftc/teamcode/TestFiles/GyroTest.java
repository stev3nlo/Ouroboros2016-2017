package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorAdafruitIMU;

/**
 * Created by Morgannanez on 10/10/16.
 */

@Autonomous(name="GyroTest", group="Test")
public class GyroTest extends MyOpMode {

    SensorAdafruitIMU gyroTest;

    public void initialize(){
		telemetry.addData("Gyro", "Initializing");
		telemetry.update();
        gyroTest = new SensorAdafruitIMU(hardwareMap.get(BNO055IMU.class, "gyro"));
        telemetry.addData("Gyro", "Initialized");
		telemetry.update();
    }

    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

		while (opModeIsActive()) {
			telemetry.addData("Yaw", gyroTest.getYaw());
			telemetry.addData("Roll", gyroTest.getRoll());
			telemetry.addData("Pitch", gyroTest.getPitch());
            telemetry.update();

			idle();
		}
    }
}