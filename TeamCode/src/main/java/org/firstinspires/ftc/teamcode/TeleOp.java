package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RPMStabilizer;

/**
 * Created by Morgannanez on 9/27/16.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Teleop", group="Teleop")  // @Autonomous(...) is the other common choice
public class TeleOp extends MyOpMode
{
    //Controller values

    double g1y1;    //left drive
    double g1y2;    //right drive
    double g1x1;
    double g1x2;
    boolean g1Lbump;
    boolean g1Rbump;
    double g1Ltrig;
    double g1Rtrig;
    boolean g1XPressed;
    boolean g1YPressed;
    boolean g1APressed;
    boolean g1BPressed;

    double g2y1;    //lift
    double g2y2;    //manipulator
    double g2x1;
    double g2x2;
    boolean g2Lbump;    //grab cap ball
    boolean g2Rbump;    //release cap ball
    double g2Ltrig;
    double g2Rtrig;
    boolean g2XPressed;     //start spinner
    boolean g2YPressed;     //stop spinner
    boolean g2APressed;     //toggle servo dropper
    boolean g2BPressed;

    long timeAtEndOfLastCycle;

    double timeSinceLastStabilization;
    double timeSinceBallsDropped;

    double curPowerOfSpinner = 0.7;

    public void updateControllerVals()
    {
        g1y1 = gamepad1.left_stick_y;
        g1y2 = gamepad1.right_stick_y;
        g1x1 = gamepad1.left_stick_x;
        g1x2 = gamepad1.right_stick_x;
        g1Lbump = gamepad1.left_bumper;
        g1Rbump = gamepad1.right_bumper;
        g1Ltrig = gamepad1.left_trigger;
        g1Rtrig = gamepad1.right_trigger;
        g1XPressed = gamepad1.x;
        g1APressed = gamepad1.a;
        g1YPressed = gamepad1.y;
        g1BPressed = gamepad1.b;

        g2y1 = gamepad2.left_stick_y;
        g2y2 = gamepad2.right_stick_y;
        g2x1 = gamepad2.left_stick_x;
        g2x2 = gamepad2.right_stick_x;
        g2Lbump = gamepad2.left_bumper;
        g2Rbump = gamepad2.right_bumper;
        g2Ltrig = gamepad2.left_trigger;
        g2Rtrig = gamepad2.right_trigger;
        g2XPressed = gamepad2.x;
        g2APressed = gamepad2.a;
        g2YPressed = gamepad2.y;
        g2BPressed = gamepad2.b;
    }

    public void updateTimeVars()
    {

    }

    public void runOpMode()
    {
        while (opModeIsActive()) {
            updateControllerVals();
            updateTimeVars();
            double curTime = System.nanoTime()/1000000000;
            move(g1y1, g1y2); // moves drive wheels
            moveManip(g2y1);
            if(timeSinceLastStabilization>0.5)
            {
                double estimatedCurRPM = getSpinnerEncoderVal() - spinnerEncoderOffset;
                estimatedCurRPM /= curTime - timeAtEndOfLastCycle;
                estimatedCurRPM /= 1140;
                curPowerOfSpinner = RPMStabilizer.returnPowerToTry(curPowerOfSpinner,estimatedCurRPM,1000);
                runSpinner(curPowerOfSpinner);
            }
            if(g2APressed)
            {

            }
            timeAtEndOfLastCycle = System.nanoTime()/1000000000;
        }
    }
}