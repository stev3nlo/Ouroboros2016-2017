package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Libraries.MotorScaler;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RPMStabilizer;

import java.io.IOException;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Teleop", group="Teleop")  // @MyAutonomous(...) is the other common choice
public class TeleOp extends MyOpMode
{
    public boolean shooterIsRunning = false;
    //Controller values

    public double g1y1;    //left drive
    public double g1y2;    //right drive
    public double g1x1;
    public double g1x2;
    public boolean g1Lbump;
    public boolean g1Rbump;
    public double g1Ltrig;
    public double g1Rtrig;
    public boolean g1XPressed;
    public boolean g1YPressed;
    public boolean g1APressed;
    public boolean g1BPressed;

    int numLoopsRemainining;
    double timeAtLastLoop;
    double timePerLoop = 1.0;
    public int driverMode = 1;

    public double g2y1;    //lift
    public double g2y2;    //manipulator
    public double g2x1;
    public double g2x2;
    public boolean g2Lbump;    //grab cap ball
    public boolean g2Rbump;    //release cap ball
    public double g2Ltrig;
    public double g2Rtrig;
    public boolean g2XPressed;     //start shooter spinner
    public boolean g2YPressed;     //stop shooter spinner
    public boolean g2APressed;     //toggle servo dropper
    public boolean g2BPressed;

    long timeAtEndOfLastCycle;

    double timeBallsFinishDropping;

    public boolean firstCycleOfSpinner;

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

    public void updateTelemetry()
    {
        telemetry.addData("BEACON PUSH INFO","");
        telemetry.addData("curPowerOfMotorSpinner",curPowerOfMotorSpinner);
        if(g2Ltrig > 0.4)
        {
            telemetry.addData("g2Ltrig being pressed", "LEFT");
        }
        else if(g2Rtrig > 0.4)
        {
            telemetry.addData("g2Rtrig being pressed", "RIGHT");
        }
        else if(g2Rbump || g2Lbump)
        {
            telemetry.addData("g2Rbump or g2Lbump being pressed", "MIDDLE");
        }
        else
        {
            telemetry.addData("No beacon control being pressed","");
        }
        telemetry.addData("motorL1EncoderVal",getMotorL1EncoderVal());
        telemetry.addData("motorR1EncoderVal",getMotorR1EncoderVal());
        telemetry.update();
    }

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        //motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();

        timeAtLastLoop = getCurTime();
        while (opModeIsActive())
        {
            updateControllerVals();
            initCurtime(); //gets real time timer
            if(driverMode == 0) {
                move(g1y1, -g1y2); // moves drive wheels
                /*motorL1.setPower(1.0);
                motorL2.setPower(0.0);
                motorR1.setPower(0.0);
                motorR2.setPower(0.0);
                telemetry.addData("motorL1 being moved", "");*/
            }
            else if(driverMode == 1) {
                move(g1y1 * 0.75, -g1y2 * 0.75);
                /*motorL1.setPower(0.0);
                motorL2.setPower(1.0);
                motorR1.setPower(0.0);
                motorR2.setPower(0.0);
                telemetry.addData("motorL2 being moved", "");*/
            }
            else if(driverMode == 2) {
                move(g1y1*0.5, -g1y2*0.5);
                /*motorL1.setPower(0.0);
                motorL2.setPower(0.0);
                motorR1.setPower(1.0);
                motorR2.setPower(0.0);
                telemetry.addData("motorR1 being moved", "");*/
            }
            else if(driverMode == 3) {
                move(g1y1*0.25, -g1y2*0.25);
                /*motorL1.setPower(0.0);
                motorL2.setPower(0.0);
                motorR1.setPower(0.0);
                motorR2.setPower(1.0);
                telemetry.addData("motorR2 being moved", "");*/
            }
            if(g1APressed) {
                driverMode = 0;
            }
            else if(g1BPressed)
                driverMode = 1;
            else if(g1YPressed)
                driverMode = 2;
            else if(g1XPressed)
                driverMode = 3;
            moveManip(g2y1);

            //Stabilization

            if (g2YPressed || !shooterIsRunning)
            {
                shooterIsRunning = false;
                runSpinner(0.0);
                /*
                shooterIsRunning = false;
                firstCycleOfSpinner = true;
                if(numCyclesOfSlowingSpinner==-1)
                {
                    numCyclesOfSlowingSpinner = 20;
                    timeAtLastSpinnerSlowdown = getCurTime();
                }
                if(numCyclesOfSlowingSpinner >= 0 && getCurTime() - timeAtLastSpinnerSlowdown >= 0.3)
                {
                    runSpinner(curPowerOfMotorSpinner*((double)numCyclesOfSlowingSpinner/20.0));
                    timeAtLastSpinnerSlowdown = getCurTime();
                    if(numCyclesOfSlowingSpinner > 0)
                        numCyclesOfSlowingSpinner--;
                }
                */
            }
            //creates constant speed of spinner throughout game
            if(g2XPressed || shooterIsRunning)
            {
                runSpinner(0.9);
                /*
                shooterIsRunning = true;
                if(firstCycleOfSpinner) {
                    firstCycleOfSpinner = false;
                    initTime = getCurTime();
                    timeAtLastStabilization = getCurTime();
                    numCyclesOfSlowingSpinner = -1;
                }
                runRPMStabilization();

                runSpinner(curPowerOfMotorSpinner);
                */
            }


            //No stabilization

            /*
            if(g2YPressed || !shooterIsRunning)
            {
                runSpinner(0.0);
                shooterIsRunning = false;
            }
            if(g2XPressed || shooterIsRunning)
            {
                shooterIsRunning = true;
                runSpinner(0.35);
            }
            */



            // stops spinner


            //releases balls from basket into spinner
            if (g2APressed)// && shooterIsRunning)
            {
                openServoDropper();
            }
            else if(g2BPressed)// || !shooterIsRunning)
            {
                closeServoDropper();
            }

            if(g2Ltrig > 0.3)
            {
                moveBeaconPusherOut();
            }
            else if(g2Rtrig > 0.3)
            {
                moveBeaconPusherOut();
            }
            else if(g2Rbump || g2Lbump)
            {
                moveBeaconPusherIn();
            }
            updateTelemetry();
            //timeAtEndOfLastCycle = System.nanoTime()/1000000000;
            idle();
        }
    }
}