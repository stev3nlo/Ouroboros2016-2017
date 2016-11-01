package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RPMStabilizer;

import java.io.IOException;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Teleop", group="Teleop")  // @Autonomous(...) is the other common choice
public class TeleOp extends MyOpMode
{
    boolean shooterIsRunning = false;
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
    boolean g2XPressed;     //start shooter spinner
    boolean g2YPressed;     //stop shooter spinner
    boolean g2APressed;     //toggle servo dropper
    boolean g2BPressed;

    long timeAtEndOfLastCycle;

    double timeBallsFinishDropping;

    boolean firstCycleOfSpinner;

    double curTime;

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
        g2APressed = gamepad2.a;    //toggle servo dropper
        g2YPressed = gamepad2.y;
        g2BPressed = gamepad2.b;
    }

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        while (opModeIsActive())
        {
            updateControllerVals();
            initCurtime(); //gets real time timer
            move(g1y1, -g1y2); // moves drive wheels
            moveManip(g2y1);

            if (g2YPressed)
            {
                curPowerOfMotorSpinner = 0.0;
                shooterIsRunning = false;
                firstCycleOfSpinner = true;
            }
            //creates constant speed of spinner throughout game
            if(g2XPressed || shooterIsRunning)
            {
                shooterIsRunning = true;
                shoot();
            }

            // stops spinner


            //releases balls from basket into spinner
            if (g2APressed && curTime > timeBallsFinishDropping)
            {
                timeBallsFinishDropping = curTime + 5;
                openServoDropper();
            }
            else if (curTime > timeBallsFinishDropping)
            {
                closeServoDropper();
            }

            if(g2Ltrig > 0.1)
            {
                pushButtonLeft();
            }
            else if(g2Rtrig > 0.1)
            {
                pushButtonRight();
            }
            else if(g2Rbump || g2Lbump)
            {
                resetButtonPress();
            }
            timeAtEndOfLastCycle = System.nanoTime()/1000000000;
        }
    }
}