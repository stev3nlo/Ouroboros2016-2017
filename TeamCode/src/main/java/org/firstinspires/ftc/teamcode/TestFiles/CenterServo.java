package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

/**
 * Created by Steven on 11/7/2016.
 */

@TeleOp (name="Center Servo", group="Teleop")
public class CenterServo extends MyOpMode {

	Servo servo;
	double curVal;

	public void initialize()
	{
		servo = hardwareMap.servo.get("servo");
		servo.setPosition(.5);
		curVal = .5;
	}

	public void runOpMode() throws InterruptedException {
		initialize();
		waitForStart();
		while (opModeIsActive())
		{
			if (gamepad1.x) {	//left
				curVal = .15;
				servo.setPosition(curVal);
				telemetry.addData("Current Position", curVal);
			} else {
				if (gamepad1.a) {	//center
					curVal = .5;
					servo.setPosition(curVal);
					telemetry.addData("Current Position", curVal);
				} else {
					if (gamepad1.b)	 {	//right
						curVal = .85;
						servo.setPosition(curVal);
						telemetry.addData("Current Position", curVal);
					}
				}
			}
		}
	}
}
