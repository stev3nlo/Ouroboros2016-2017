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

	SensorAdafruitIMU gyro;

    public void initialize(){
		telemetry.addData("Gyro", "Initializing");
		telemetry.update();
        gyro = new SensorAdafruitIMU(hardwareMap.get(BNO055IMU.class, "gyro"));
        telemetry.addData("Gyro", "Initialized");
		telemetry.update();
    }

    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

		while (opModeIsActive()) {
			telemetry.addData("Yaw", gyro.getYaw());
			telemetry.addData("Roll", gyro.getRoll());
			telemetry.addData("Pitch", gyro.getPitch());
            telemetry.update();

			idle();
		}
    }
}