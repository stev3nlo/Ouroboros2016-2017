package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RPMStabilizer;

/**
 * Created by Steven on 10/20/2016.
 */
@Autonomous(name="RPM Stabilizer Test", group="Test")
public class RPMStabilizerTest extends MyOpMode {

	double power;
	double currPower;
	double currRPM;
	double goalRPM;

	public void runOpMode() {
		power = RPMStabilizer.returnPowerToTry(currPower, currRPM, goalRPM);
	}
}