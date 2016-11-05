package org.firstinspires.ftc.teamcode.TestFiles;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;

/**
 * Created by spencersharp on 11/5/16.
 */
public class TestWheelRPM extends MyOpMode
{
    TreeMap<Double,Long> motorL1RPMs;
    TreeMap<Double,Long> motorR1RPMs;
    public void runOpMode()
    {
        motorL1RPMs = new TreeMap<Double,Long>();
        initCurtime();
        try{waitForStart();}catch(InterruptedException e){}
        motorL1RPMs.put(getCurTime(), getMotorL1EncoderVal());
        motorL1RPMs.put(getCurTime(), getMotorR1EncoderVal());
        while(opModeIsActive())
        {
            initCurtime();
            HashMap<Double,Long> firstEncoderTimeSetAfterTimeL1 = getFirstEncoderTimeSetAfterTimeL1(getCurTime()-5.0);
            HashMap<Double,Long> firstEncoderTimeSetAfterTimeR1 = getFirstEncoderTimeSetAfterTimeR1(getCurTime()-5.0);
            Map.Entry pairL1 = getFirstSetFromHashMap(firstEncoderTimeSetAfterTimeL1);
            double oldTimeL1 = (double)pairL1.getKey();
            long oldEncoderValL1 = (long)pairL1.getValue();
            Map.Entry pairR1 = getFirstSetFromHashMap(firstEncoderTimeSetAfterTimeR1);
            double oldTimeR1 = (double)pairR1.getKey();
            long oldEncoderValR1 = (long)pairR1.getValue();
            double calcedRPMOfMotorL1 = getMotorL1EncoderVal() - oldEncoderValL1;
            calcedRPMOfMotorL1 /= getCurTime()-oldTimeL1;
            double calcedRPMOfMotorR1 = getMotorR1EncoderVal() - oldEncoderValR1;
            calcedRPMOfMotorR1 /= getCurTime()-oldTimeR1;
            telemetry.addData("motorL1RPM",calcedRPMOfMotorL1);
            telemetry.addData("motorR1RPM", calcedRPMOfMotorR1);
            try{idle();}catch(InterruptedException e){}
        }
    }

    public Map.Entry getFirstSetFromHashMap(HashMap<Double,Long> single)
    {
        Iterator it = single.entrySet().iterator();
        ArrayList<Double> toRemove = new ArrayList<Double>();
        while (it.hasNext()) {
            Map.Entry pair = (Map.Entry)it.next();
            return pair;
        }
    }

    public HashMap<Double,Long> getFirstEncoderTimeSetAfterTimeL1(double t)
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
            motorL1RPMs.remove(toRemove);
        return vals;
    }

    public HashMap<Double,Long> getFirstEncoderTimeSetAfterTimeR1(double t)
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
            motorR1RPMs.remove(toRemove);
        return vals;
    }
}
