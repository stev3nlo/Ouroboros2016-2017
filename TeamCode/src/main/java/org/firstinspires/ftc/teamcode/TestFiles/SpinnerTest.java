package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.Libraries.MotorScaler;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RPMStabilizer;
import org.firstinspires.ftc.teamcode.TeleOp;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;

/**
 * Created by Spencer on 10/20/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Spinner Test", group="Test")
public class SpinnerTest extends MyOpMode {

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

    double timeAtLastRPMUpdate;
    int motorL1StartEncoder;
    int motorL2StartEncoder;
    int motorR1StartEncoder;
    int motorR2StartEncoder;
    int motorSpinnerStartEncoder;

    double curRPM;

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

    public double calculateRPM(String motorName)
    {
        double startEncoder;
        double curEncoder;
        double timePassed;
        if(motorName.equals("motorL1"))
        {
            startEncoder = motorL1StartEncoder;
            curEncoder = motorL1.getCurrentPosition();
        }
        else if(motorName.equals("motorL2"))
        {
            startEncoder = motorL2StartEncoder;
            curEncoder = motorL2.getCurrentPosition();
        }
        else if(motorName.equals("motorR1"))
        {
            startEncoder = motorR1StartEncoder;
            curEncoder = motorR1.getCurrentPosition();
        }
        else if(motorName.equals("motorR2"))
        {
            startEncoder = motorR2StartEncoder;
            curEncoder = motorR2.getCurrentPosition();
        }
        else
        {
            startEncoder = motorSpinnerStartEncoder;
            curEncoder = motorSpinner.getCurrentPosition();
        }
        initCurtime();
        double thisTime = getCurTime();
        timePassed = thisTime - timeAtLastRPMUpdate;
        double RPM = (curEncoder - startEncoder) / timePassed;
        RPM /= 1120;
        return RPM;
    }

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        waitForStart();
        double speed = 0.0;
        initCurtime();
        timeAtLastRPMUpdate = getCurTime();
        motorSpinnerStartEncoder = motorSpinner.getCurrentPosition();
        while(opModeIsActive())
        {
            updateControllerVals();
            if(g1Ltrig > 0.3 || g1Rtrig > 0.3)
            {
                openServoDropper();
            }
            else if(g1Rbump || g1Lbump)
                closeServoDropper();
            if(g1XPressed)
            {
                speed += 0.01;
            }
            else if(g1YPressed)
            {
                speed -= 0.01;
            }
            else if(g1APressed)
            {
                speed += 0.1;
            }
            else if(g1BPressed)
            {
                speed -= 0.1;
            }
            if(speed < 0.0)
                speed = 0.0;
            else if(speed > 1.0)
                speed = 1.0;
            runSpinner(speed);
            initCurtime();
            if(timeAtLastRPMUpdate + 5.0 < getCurTime())
            {
                curRPM = calculateRPM("motorSpinner");
                motorSpinnerStartEncoder = motorSpinner.getCurrentPosition();
                timeAtLastRPMUpdate = getCurTime();
            }
            telemetry.addData("timeAtLastRPMUpdate",timeAtLastRPMUpdate);
            telemetry.addData("curTime",getCurTime());
            telemetry.addData("RPM",curRPM);
            telemetry.addData("speed", speed);
            telemetry.update();
            pause(0.25);
            idle();
        }
    }

}