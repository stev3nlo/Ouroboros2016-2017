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
public class SpinnerTest extends TeleOp {

    double targetRPM = 500;
    double curFactor;
    double timeAtLastRPMUpdate;
    double timeAtLastButtonPress;
    double timeAtLastTriggerPress;
    int targetTicksPerSecond;
    double curPower;
    boolean isStopped = true;

    TreeMap<Double,Long> RPMs;

    //DcMotor motorSpinner;

    public void initialize()
    {
        targetTicksPerSecond = ((int)(((double)(encoderTicksPerRotation*targetRPM))/60.0));
        motorSpinner = hardwareMap.dcMotor.get("motorSpinner");
        motorSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSpinner.setMaxSpeed(targetTicksPerSecond);
        curFactor = 100;
        curPower = 1.0;
    }

    public void runTelemetry()
    {
        telemetry.addData("curFactor",curFactor);
        telemetry.addData("targetTicksPerSecond",targetTicksPerSecond);
        telemetry.addData("encoderTicks",getSpinnerEncoderVal());
        telemetry.addData("targetRPM",targetRPM);
        telemetry.addData("curPower",curPower);
        telemetry.addData("timeSinceLastTimeButtonPress ", getCurTime() - timeAtLastButtonPress);
        telemetry.addData("timeSinceLastTriggerPress", getCurTime() - timeAtLastTriggerPress);
        telemetry.addData("curTime",getCurTime());
        telemetry.update();
    }

    /*

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

    }
    */

    public void checkButtons()
    {
        if(g1YPressed && getCurTime() - timeAtLastButtonPress > 0.2)
        {
            timeAtLastButtonPress = getCurTime();
            curFactor *= 10;
        }
        else if(g1APressed && getCurTime() - timeAtLastButtonPress > 0.2)
        {
            timeAtLastButtonPress = getCurTime();
            curFactor /= 10;
        }
        else if(isStopped && g1BPressed && getCurTime() - timeAtLastButtonPress > 0.2)
        {
            timeAtLastButtonPress = getCurTime();
            curPower = 1.0;
        }
        else if(!isStopped && g1BPressed && getCurTime() - timeAtLastButtonPress > 0.2)
        {
            timeAtLastButtonPress = getCurTime();
            curPower = 0.0;
        }
    }

    /*
    public void capPower()
    {
        if(curPower < -1.0)
            curPower = -1.0;
        else if(curPower > 1.0)
            curPower = 1.0;
    }
    */

    public Map.Entry getFirstSetFromHashMap(HashMap<Double,Long> single)
    {
        Iterator it = single.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry pair = (Map.Entry)it.next();
            return pair;
        }
        return null;
    }

    public HashMap<Double,Long> getFirstEncoderTimeSetAfterTimeL1(double t)
    {
        HashMap<Double,Long> vals = new HashMap<Double,Long>();
        Iterator it = RPMs.entrySet().iterator();
        ArrayList<Double> toRemove = new ArrayList<Double>();
        while (it.hasNext()) {
            Map.Entry pair = (Map.Entry)it.next();
            if((double)pair.getKey()>t)
            {
                vals.put((double)pair.getKey(),(long)pair.getValue());
                break;
            }
            else
            {
                toRemove.add((double)pair.getKey());
            }
            it.remove(); // avoids a ConcurrentModificationException
        }
        while(toRemove.size()>0)
            RPMs.remove(toRemove);
        return vals;
    }

    public void checkTriggers()
    {
        if(g1Rtrig > 0.1 && getCurTime() - timeAtLastTriggerPress > 0.2)
        {
            targetRPM += curFactor;
            timeAtLastTriggerPress = getCurTime();
        }
        else if(g1Rbump && getCurTime() - timeAtLastTriggerPress > 0.2)
        {
            targetRPM -= curFactor;
            timeAtLastTriggerPress = getCurTime();
        }
    }

    public void runOpMode()
    {
        initialize();
        try{waitForStart();}catch(InterruptedException e){}
        initCurtime();
        timeAtLastRPMUpdate = getCurTime();
        timeAtLastButtonPress = getCurTime();
        timeAtLastTriggerPress = getCurTime();
        while(opModeIsActive())
        {
            updateControllerVals();
            initCurtime();
            checkButtons();
            checkTriggers();
            targetTicksPerSecond = ((int)(((double)(encoderTicksPerRotation*targetRPM))/60.0));
            motorSpinner.setMaxSpeed(targetTicksPerSecond);
            motorSpinner.setPower(curPower);
            runTelemetry();
        }
    }
}