package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

/**
 * Created by Steven on 11/11/2016.
 */

@Autonomous(name="AutonomousBlue", group="Auto")
public class AutonomousBlue extends MyAutonomous {

	@Override
	public void runOpMode() throws InterruptedException {
		super.runOpMode();
		waitForStart();
		initCurtime();
		initTime = getCurTime();

		//starts spinner
		simpleStabilizingLoop(1.0);

		//move with range sensors to right location

		//moveWithEncoders(.8, 4200);
		simpleStabilizingLoop(1.0);
		openServoDropper();
		initTime = getCurTime();
		simpleStabilizingLoop(1.5);
		closeServoDropper();

		moveWithEncoders(.8, 3700,.93,1);
		double degreesToTurn = 0.0;
		//double degreesToTurn = getDegreesToTurnFromDistances(rangeF.getDistance(DistanceUnit.CM),rangeB.getDistance(DistanceUnit.CM));
		if(degreesToTurn > 0)
			gyroTurnRightCorrection(0.6,degreesToTurn);
		else
			gyroTurnLeftCorrection(0.6,degreesToTurn);
		//moveAlongWallToBeacon(0.7,4.0,true);
		pushButton();

	}
}
