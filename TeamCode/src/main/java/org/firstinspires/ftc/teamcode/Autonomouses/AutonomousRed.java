package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

/**
 * Created by Steven on 11/11/2016.
 */

@Autonomous(name="AutonomousRed", group="Auto")
public class AutonomousRed extends MyAutonomous
{
	@Override
	public void runOpMode() throws InterruptedException {
		super.runOpMode();
		waitForStart();
		initCurtime();
		initTime = getCurTime();
		simpleStabilizingLoop(1.0);
		moveWithEncoders(-.8, 3100);
		simpleStabilizingLoop(1.0);
		openServoDropper();
		initTime = getCurTime();
		simpleStabilizingLoop(1.5);
		closeServoDropper();

		moveWithEncoders(-.8, 3300);
		//double degreesToTurn = getDegreesToTurnFromDistances(rangeB.getDistance(DistanceUnit.CM),rangeF.getDistance(DistanceUnit.CM));
		double degreesToTurn = 26.0;
		telemetry.addData("degreesToTurn",degreesToTurn);
		//telemetry.addData("rangeBDist",rangeB.getDistance(DistanceUnit.CM));
		//telemetry.addData("rangeFDist",rangeF.getDistance(DistanceUnit.CM));
		telemetry.addData("turning", "");
		telemetry.update();
		gyroTurnRight(0.3, degreesToTurn);
		pause(1.0);
		telemetry.addData("done turning","");
		telemetry.addData("moving along wall to beacon","");
		telemetry.update();
		moveAlongWallToBeacon(-0.23, 4.0, false);
		pushButton();

	}
}
