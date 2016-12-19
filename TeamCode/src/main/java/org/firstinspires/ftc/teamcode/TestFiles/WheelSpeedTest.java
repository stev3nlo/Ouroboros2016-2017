package org.firstinspires.ftc.teamcode.TestFiles;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;

/**
 * Created by spencersharp on 12/19/16.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="WheelSpeedTest", group="Test")
public class WheelSpeedTest extends MyOpMode
{
    public double curPowerOfMotorL1 = 1.0;
    public double curPowerOfMotorL2 = 1.0;
    public double curPowerOfMotorR1 = 1.0;
    public double curPowerOfMotorR2 = 1.0;

    protected TreeMap<Double,Long> motorL1RPMs = new TreeMap<Double,Long>();
    protected TreeMap<Double,Long> motorL2RPMs = new TreeMap<Double,Long>();
    protected TreeMap<Double,Long> motorR1RPMs = new TreeMap<Double,Long>();
    protected TreeMap<Double,Long> motorR2RPMs = new TreeMap<Double,Long>();
    public HashMap<Double,Long> getFirstEncoderTimeSetAfterTime(double t, String name)
    {
        if(name.equals("motorL1"))
        {
            HashMap<Double,Long> vals = new HashMap<Double,Long>();
            Iterator it = motorL1RPMs.entrySet().iterator();
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
                motorL1RPMs.remove(toRemove.remove(0));
            return vals;
        }
        else if(name.equals("motorR1"))
        {
            HashMap<Double,Long> vals = new HashMap<Double,Long>();
            Iterator it = motorR1RPMs.entrySet().iterator();
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
                motorR1RPMs.remove(toRemove.remove(0));
            return vals;
        }
        else if(name.equals("motorL2"))
        {
            HashMap<Double,Long> vals = new HashMap<Double,Long>();
            Iterator it = motorL2RPMs.entrySet().iterator();
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
                motorL2RPMs.remove(toRemove.remove(0));
            return vals;
        }
        else
        {
            HashMap<Double,Long> vals = new HashMap<Double,Long>();
            Iterator it = motorR2RPMs.entrySet().iterator();
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
                motorR2RPMs.remove(toRemove.remove(0));
            return vals;
        }
    }

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        initCurtime();
        timeAtLastStabilization = getCurTime() +5.0;
        motorL1.setPower(-motorL1SpeedMultiplier);
        motorL2.setPower(-motorL2SpeedMultiplier);
        motorR1.setPower(motorR1SpeedMultiplier);
        motorR2.setPower(motorR2SpeedMultiplier);
        while(opModeIsActive())
        {
            initCurtime();
            motorL1RPMs.put(getCurTime(), getMotorL1EncoderVal());
            motorL2RPMs.put(getCurTime(), getMotorL2EncoderVal());
            motorR1RPMs.put(getCurTime(), getMotorR1EncoderVal());
            motorR2RPMs.put(getCurTime(), getMotorR2EncoderVal());
            if(getCurTime()-timeAtLastStabilization > 5.0)
            {
                HashMap<Double, Long> firstEncoderTimeSetAfterTimeL1 = getFirstEncoderTimeSetAfterTime(getCurTime() - 5.0, "motorL1");
                Map.Entry pairL1 = getFirstSetFromHashMap(firstEncoderTimeSetAfterTimeL1);
                oldTime = (double) pairL1.getKey();
                oldEncoderVal = (long) pairL1.getValue();
                curRPM = getMotorL1EncoderVal() - oldEncoderVal;
                curRPM /= 1120; //Number of rotations since last run
                timeSinceLastRPMUpdate = getCurTime() - oldTime;
                curRPM /= getCurTime() - oldTime; //Number of rotations per second
                curRPM *= 60.0;
                telemetry.addData("motorL1RPM",curRPM);
                //telemetry.addData("motorRPM", curRPM);
                /*
                if (curRPM > goalRPM)
                    curPowerOfMotorL1 -= Math.sqrt(Math.abs(((curRPM - goalRPM) * (curRPM - goalRPM) * (curRPM - goalRPM)))) / (10000);
                else
                    curPowerOfMotorL1 += Math.sqrt(Math.abs(((curRPM - goalRPM) * (curRPM - goalRPM) * (curRPM - goalRPM)))) / (10000);
                if (curPowerOfMotorL1 > 1.0)
                    curPowerOfMotorL1 = 1.0;
                else if (curPowerOfMotorL1 < 0.0)
                    curPowerOfMotorL1 = 0.0;
                */


                HashMap<Double, Long> firstEncoderTimeSetAfterTimeL2 = getFirstEncoderTimeSetAfterTime(getCurTime() - 6.0, "motorL2");
                Map.Entry pairL2 = getFirstSetFromHashMap(firstEncoderTimeSetAfterTimeL2);
                oldTime = (double) pairL2.getKey();
                oldEncoderVal = (long) pairL2.getValue();
                curRPM = getMotorL2EncoderVal() - oldEncoderVal;
                curRPM /= 1120; //Number of rotations since last run
                timeSinceLastRPMUpdate = getCurTime() - oldTime;
                curRPM /= getCurTime() - oldTime; //Number of rotations per second
                curRPM *= 60.0;
                telemetry.addData("motorL2RPM",curRPM);
                //telemetry.addData("motorRPM", curRPM);
                /*
                if (curRPM > goalRPM)
                    curPowerOfMotorL2 -= Math.sqrt(Math.abs(((curRPM - goalRPM) * (curRPM - goalRPM) * (curRPM - goalRPM)))) / (10000);
                else
                    curPowerOfMotorL2 += Math.sqrt(Math.abs(((curRPM - goalRPM) * (curRPM - goalRPM) * (curRPM - goalRPM)))) / (10000);
                if (curPowerOfMotorL2 > 1.0)
                    curPowerOfMotorL2 = 1.0;
                else if (curPowerOfMotorL2 < 0.0)
                    curPowerOfMotorL2 = 0.0;
                */


                HashMap<Double, Long> firstEncoderTimeSetAfterTimeR1 = getFirstEncoderTimeSetAfterTime(getCurTime() - 6.0, "motorR1");
                Map.Entry pairR1 = getFirstSetFromHashMap(firstEncoderTimeSetAfterTimeR1);
                oldTime = (double) pairR1.getKey();
                oldEncoderVal = (long) pairR1.getValue();
                curRPM = getMotorR1EncoderVal() - oldEncoderVal;
                curRPM /= 1120; //Number of rotations since last run
                timeSinceLastRPMUpdate = getCurTime() - oldTime;
                curRPM /= getCurTime() - oldTime; //Number of rotations per second
                curRPM *= 60.0;
                telemetry.addData("motorR1RPM",curRPM);
                //telemetry.addData("motorRPM", curRPM);
                /*
                if (curRPM > goalRPM)
                    curPowerOfMotorL1 -= Math.sqrt(Math.abs(((curRPM - goalRPM) * (curRPM - goalRPM) * (curRPM - goalRPM)))) / (10000);
                else
                    curPowerOfMotorL1 += Math.sqrt(Math.abs(((curRPM - goalRPM) * (curRPM - goalRPM) * (curRPM - goalRPM)))) / (10000);
                if (curPowerOfMotorL1 > 1.0)
                    curPowerOfMotorL1 = 1.0;
                else if (curPowerOfMotorL1 < 0.0)
                    curPowerOfMotorL1 = 0.0;
                */


                HashMap<Double, Long> firstEncoderTimeSetAfterTimeR2 = getFirstEncoderTimeSetAfterTime(getCurTime() - 6.0, "motorR2");
                Map.Entry pairR2 = getFirstSetFromHashMap(firstEncoderTimeSetAfterTimeL2);
                oldTime = (double) pairR2.getKey();
                oldEncoderVal = (long) pairR2.getValue();
                curRPM = getMotorR2EncoderVal() - oldEncoderVal;
                curRPM /= 1120; //Number of rotations since last run
                timeSinceLastRPMUpdate = getCurTime() - oldTime;
                curRPM /= getCurTime() - oldTime; //Number of rotations per second
                curRPM *= 60.0;
                telemetry.addData("motorR2RPM",curRPM);
                //telemetry.addData("motorRPM", curRPM);
                /*
                if (curRPM > goalRPM)
                    curPowerOfMotorL1 -= Math.sqrt(Math.abs(((curRPM - goalRPM) * (curRPM - goalRPM) * (curRPM - goalRPM)))) / (10000);
                else
                    curPowerOfMotorL1 += Math.sqrt(Math.abs(((curRPM - goalRPM) * (curRPM - goalRPM) * (curRPM - goalRPM)))) / (10000);
                if (curPowerOfMotorL1 > 1.0)
                    curPowerOfMotorL1 = 1.0;
                else if (curPowerOfMotorL1 < 0.0)
                    curPowerOfMotorL1 = 0.0;
                */
                telemetry.update();


                motorL1.setPower(-motorL1SpeedMultiplier);
                motorL2.setPower(-motorL2SpeedMultiplier);
                motorR1.setPower(motorR1SpeedMultiplier);
                motorR2.setPower(motorR2SpeedMultiplier);


                timeAtLastStabilization = getCurTime();
                idle();
            }
        }
    }

}
