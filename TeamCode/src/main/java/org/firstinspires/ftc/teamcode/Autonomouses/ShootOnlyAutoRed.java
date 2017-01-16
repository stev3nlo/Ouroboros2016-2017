package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

/**
 * Created by Steven on 11/11/2016.
 */

@Autonomous(name="Shoot Only Auto Red", group="Auto")
public class ShootOnlyAutoRed extends MyAutonomous
{
	@Override
	public void runOpMode() throws InterruptedException {
		super.runOpMode();
		waitForStart();
		initCurtime();
		initTime = getCurTime();
		simpleStabilizingLoop(1.0);
		moveWithEncoders(-.8, 3700,.93,1);
		simpleStabilizingLoop(1.0);
		openServoDropper();
		initTime = getCurTime();
		simpleStabilizingLoop(1.5);
		closeServoDropper();
		/*
		moveWithEncoders(-.8, 2800);
		double degreesToTurn = getDegreesToTurnFromDistances(rangeB.getDistance(DistanceUnit.CM),rangeF.getDistance(DistanceUnit.CM));
		if(degreesToTurn > 0)
			gyroTurnRightCorrection(0.6,degreesToTurn);
		else
			gyroTurnLeftCorrection(0.6,degreesToTurn);
		moveAlongWallToBeacon(0.7, 4.0, true);
		pushButton();
		*/
	}
}
