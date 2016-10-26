package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.Libraries.MotorScaler;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RPMStabilizer;

/**
 * Created by Spencer on 10/20/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Spinner Test", group="Test")
public class SpinnerTest extends TeleOp {

    double curFactor;
    double curPower;
    double curRPM;
    double timeAtLastRPMUpdate;
    double timeAtLastButtonPress;
    double timeAtLastTriggerPress;

    //DcMotor motorSpinner;

    public void initialize()
    {
        motorSpinner = hardwareMap.dcMotor.get("motorSpinner");
        curFactor = 0.1;
        curRPM = 0.0;
        curPower = 0.8;

    }

    public void runTelemetry()
    {
        telemetry.addData("curFactor",curFactor);
        telemetry.addData("curPower",curPower);
        telemetry.addData("scaledPower", MotorScaler.reverseScale(curPower));
        telemetry.addData("curRPM",curRPM);
        telemetry.addData("timeSinceLastButtonPress",curTime-timeAtLastButtonPress);
        telemetry.addData("timeSinceLastTriggerPress",curTime-timeAtLastTriggerPress);
        telemetry.addData("curTime",curTime);
        telemetry.update();
    }

    public double getCurRPM()
    {
        if(curTime-timeAtLastRPMUpdate>0.5)
        {
            double estimatedCurRPM = getSpinnerEncoderVal() - spinnerEncoderOffset; // gets current ticks
            spinnerEncoderOffset = getSpinnerEncoderVal();

            estimatedCurRPM /= curTime - timeAtLastRPMUpdate;   // gets time
            estimatedCurRPM /= 1140;

            timeAtLastRPMUpdate = curTime;

            return estimatedCurRPM;
        }
        return curRPM;
        /*
        if(firstCycleOfSpinner)
        {
            firstCycleOfSpinner = false;
            timeAtLastStabilization = curTime;
            spinnerEncoderOffset = getSpinnerEncoderVal();
            runSpinner(curPowerOfSpinner);
        }
        else if (curTime - timeAtLastStabilization > 0.5)
        {
            double estimatedCurRPM = getSpinnerEncoderVal() - spinnerEncoderOffset; // gets current ticks
            spinnerEncoderOffset = getSpinnerEncoderVal();

            estimatedCurRPM /= curTime - timeAtLastStabilization;   // gets time
            timeAtLastStabilization = curTime;
            estimatedCurRPM /= 1140;

            curPowerOfSpinner = RPMStabilizer.returnPowerToTry(curPowerOfSpinner, estimatedCurRPM, 700);
            runSpinner(curPowerOfSpinner);
        }
        */
    }

    public void checkButtons()
    {
        if(g1YPressed && curTime - timeAtLastButtonPress > 0.2)
        {
            timeAtLastButtonPress = curTime;
            curFactor *= 10;
        }
        else if(g1APressed && curTime - timeAtLastButtonPress > 0.2)
        {
            timeAtLastButtonPress = curTime;
            curFactor /= 10;
        }
    }

    public void capPower()
    {
        if(curPower < -1.0)
            curPower = -1.0;
        else if(curPower > 1.0)
            curPower = 1.0;
    }

    public void checkTriggers()
    {
        if(g1Rtrig > 0.1 && curTime - timeAtLastTriggerPress > 0.2)
        {
            curPower += curFactor;
            timeAtLastTriggerPress = curTime;
        }
        else if(g1Rbump && curTime - timeAtLastTriggerPress > 0.2)
        {
            curPower -= curFactor;
            timeAtLastTriggerPress = curTime;
        }
    }

    public void initCurtime()
    {
        curTime = ((double)System.nanoTime())/1000000000.0;
    }

    public void runOpMode()
    {
        initialize();
        try{waitForStart();}catch(InterruptedException e){}
        initCurtime();
        timeAtLastRPMUpdate = curTime;
        timeAtLastButtonPress = curTime;
        timeAtLastTriggerPress = curTime;
        while(opModeIsActive()) {
            updateControllerVals();
            if(g1BPressed)
                curPower = 0;
            initCurtime();
            checkButtons();
            checkTriggers();
            curRPM = getCurRPM();
            capPower();
            motorSpinner.setPower(-curPower);
            runTelemetry();

        }
    }
}