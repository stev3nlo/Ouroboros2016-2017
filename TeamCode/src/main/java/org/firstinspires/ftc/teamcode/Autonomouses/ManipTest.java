package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

/**
 * Created by Steven on 11/11/2016.
 */

@Autonomous(name="Manip Test", group="Test")
public class ManipTest extends MyOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        motorManip.setPower(-1.0);
        while(opModeIsActive())
        {
            idle();
        }
		/*
		moveWithEncoders(.8, 3700);
		double degreesToTurn = getDegreesToTurnFromDistances(rangeF.getDistance(DistanceUnit.CM),rangeB.getDistance(DistanceUnit.CM));
		if(degreesToTurn > 0)
			gyroTurnRightCorrection(0.6,degreesToTurn);
		else
			gyroTurnLeftCorrection(0.6,degreesToTurn);
		moveAlongWallToBeacon(0.7,4.0,true);
		pushButton();
		*/
    }
}
