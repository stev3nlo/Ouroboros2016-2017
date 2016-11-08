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

    double targetRPM = 55;
    double curFactor;
    double timeAtLastRPMUpdate;
    double timeAtLastButtonPress;
    double timeAtLastTriggerPress;
    int targetTicksPerSecond;
    double calcedRPMOfMotor;
    boolean isStopped = true;

    TreeMap<Double,Long> RPMs = new TreeMap<Double,Long>();

    //DcMotor motorSpinner;

    public void initialize()
    {
        targetTicksPerSecond = ((int)(((double)(encoderTicksPerRotation*targetRPM))/60.0));
        motorSpinner = hardwareMap.dcMotor.get("motorSpinner");
        //motorSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorSpinner.setMaxSpeed(targetTicksPerSecond);
        curFactor = 100;
        curPowerOfMotorSpinner = 0.3;
    }

    public void runTelemetry()
    {
        telemetry.addData("calcedRPMOfMotor",calcedRPMOfMotor);
        telemetry.addData("curFactor",curFactor);
        telemetry.addData("targetTicksPerSecond",targetTicksPerSecond);
        telemetry.addData("encoderTicks",getSpinnerEncoderVal());
        telemetry.addData("targetRPM",targetRPM);
        telemetry.addData("curPower",curPowerOfMotorSpinner);
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
            curPowerOfMotorSpinner = 1.0;
        }
        else if(!isStopped && g1BPressed && getCurTime() - timeAtLastButtonPress > 0.2)
        {
            timeAtLastButtonPress = getCurTime();
            //curPowerOfMotorSpinner = 0.0;
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

    public Map.Entry getFirstSetFromHashMap(HashMap<Double,Long> single)
    {
        Iterator it = single.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry pair = (Map.Entry)it.next();
            return pair;
        }
        return null;
    }

    public HashMap<Double,Long> getFirstEncoderTimeSetAfterTime(double t)
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
            RPMs.remove(toRemove.remove(0));
        return vals;
    }

    public void runOpMode()
    {
        initialize();
        try{waitForStart();}catch(InterruptedException e){}
        initCurtime();
        timeAtLastRPMUpdate = getCurTime();
        timeAtLastButtonPress = getCurTime();
        timeAtLastTriggerPress = getCurTime();
        RPMs.put(getCurTime(),getSpinnerEncoderVal());
        double initTime = getCurTime();
        while(opModeIsActive())
        {
            updateControllerVals();
            initCurtime();
            checkButtons();
            checkTriggers();
            targetTicksPerSecond = ((int)(((double)(encoderTicksPerRotation*targetRPM))/60.0));
            RPMs.put(getCurTime(),getSpinnerEncoderVal());
            if(getCurTime() > initTime + 0.35) {
                HashMap<Double, Long> firstEncoderTimeSetAfterTime = getFirstEncoderTimeSetAfterTime(getCurTime() - 0.3);
                Map.Entry pair = getFirstSetFromHashMap(firstEncoderTimeSetAfterTime);
                double oldTime = (double) pair.getKey();
                long oldEncoderVal = (long) pair.getValue();
                calcedRPMOfMotor = getSpinnerEncoderVal() - oldEncoderVal;
                calcedRPMOfMotor /= 1120; //Number of rotations since last run
                calcedRPMOfMotor /= getCurTime() - oldTime; //Number of rotations per second
                calcedRPMOfMotor *= 60.0;
                telemetry.addData("motorRPM", calcedRPMOfMotor);
                if(calcedRPMOfMotor>targetRPM)
                    curPowerOfMotorSpinner -= Math.sqrt(Math.abs(((calcedRPMOfMotor - targetRPM)*(calcedRPMOfMotor-targetRPM)*(calcedRPMOfMotor-targetRPM)))) / (750000.0*25);
                else
                    curPowerOfMotorSpinner += Math.sqrt(Math.abs(((calcedRPMOfMotor - targetRPM)*(calcedRPMOfMotor-targetRPM)*(calcedRPMOfMotor-targetRPM)))) / (750000.0*25);
                if (curPowerOfMotorSpinner > 1.0)
                    curPowerOfMotorSpinner = 1.0;
                else if (curPowerOfMotorSpinner < 0.0)
                    curPowerOfMotorSpinner = 0.0;
                initTime = getCurTime();
            }
            motorSpinner.setPower(curPowerOfMotorSpinner);
            runTelemetry();
            try{idle();}catch(InterruptedException e){}
        }
    }
}