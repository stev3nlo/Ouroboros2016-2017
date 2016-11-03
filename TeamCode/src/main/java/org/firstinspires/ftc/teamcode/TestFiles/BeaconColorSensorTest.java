package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;

/**
 * Created by Steven on 11/1/2016.
 */
@Autonomous(name = "BeaconColorSensorTest", group = "Test")
public class BeaconColorSensorTest extends MyOpMode {

	public void initialize(){
		colorB = new SensorMRColor(hardwareMap.colorSensor.get("colorB"));

		colorB.sensorSetup(0x2c);
		colorB.lightOff();

		servoBeaconPusher = hardwareMap.servo.get("servoBeaconPusher");
		servoDropper = hardwareMap.servo.get("servoDropper");
		/*
		while (opModeIsActive())
		{
			pushButtonLeft();
			initCurtime();
			double startTime = getCurTime();
			while(getCurTime() < startTime + 0.5)
			{
				initCurtime();
				try{idle();}catch(InterruptedException e){}
			}
			pushButtonRight();
			initCurtime();
			startTime = getCurTime();
			while(getCurTime() < startTime + 0.5)
			{
				initCurtime();
				try{idle();}catch(InterruptedException e){}
			}
		}
		*/
		resetButtonPress();
		servoDropper.setPosition(1);
	}

	public void runOpMode() throws InterruptedException {
		initialize();
		waitForStart();
//		while(opModeIsActive())
//		{
//			telemetry.addData("Color Center Values", colorB);
//			telemetry.addData("Color Center Beacon", colorB.beaconColor());
//			telemetry.update();
//			idle();
//		}
		initCurtime();
		pushButtonRight();
		initCurtime();
		double startTime = getCurTime();
		while(getCurTime() < startTime + 2.0)
		{
			initCurtime();
			idle();
		}
		//Thread.sleep(250);
		pushButtonLeft();
		initCurtime();
		startTime = getCurTime();
		while (getCurTime() < startTime + 2.0)
		{
			initCurtime();
			idle();
		}
		resetButtonPress();
		initCurtime();
		startTime = getCurTime();
		while (getCurTime() < startTime + 2.0)
		{
			initCurtime();
			idle();
		}
	}
}
