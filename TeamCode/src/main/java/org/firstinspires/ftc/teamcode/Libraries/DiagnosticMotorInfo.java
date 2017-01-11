package org.firstinspires.ftc.teamcode.Libraries;

import java.io.Serializable;
import static java.lang.System.out;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

public class DiagnosticMotorInfo implements Serializable
{
    public String motorName;
    public HashMap<Integer,HashMap<Double,DiagnosticData>> recordedData;

    public DiagnosticMotorInfo()
    {
        recordedData = new HashMap<Integer,HashMap<Double,DiagnosticData>>();
    }
    public DiagnosticMotorInfo(String motorName, HashMap<Integer,HashMap<Double,DiagnosticData>> recordedData)
    {
        this.motorName = motorName;
        this.recordedData = recordedData;
    }

    public String getMotorName()
    {
        return motorName;
    }

    public void add(DiagnosticData d)
    {
        HashMap<Double,DiagnosticData> hmForPower;
        hmForPower = recordedData.get(d.getBatteryVoltage());
        if(hmForPower==null)
            hmForPower = new HashMap<Double,DiagnosticData>();
        out.println(hmForPower);
        hmForPower.put(d.getPowerSent(),d);
        recordedData.put(d.getBatteryVoltage(),hmForPower);
    }

    public HashMap<Double,DiagnosticData> getHashMapOfDiagnosticDataFromVoltage(int batteryVoltage)
    {
        //out.println(batteryVoltage);
        HashMap<Double, DiagnosticData> ret = recordedData.get(batteryVoltage);

        if(ret==null)
        {
            return new HashMap<Double,DiagnosticData>();
        }
        return ret;
    }

    public String toString()
    {
        String s = "[MOTORINFONAME" + motorName;
        Iterator it = recordedData.entrySet().iterator();
        while (it.hasNext())
        {
            Map.Entry pair = (Map.Entry)it.next();
            HashMap<Double,DiagnosticData> curHM = ((HashMap<Double,DiagnosticData>) pair.getValue());
            Iterator it2 = curHM.entrySet().iterator();
            while(it2.hasNext())
            {
                Map.Entry pair2 = (Map.Entry)it2.next();
                s += "{" + (DiagnosticData)pair2.getValue() + "}\n";
            }
            //it.remove();
        }
        s+="]";
        return s;
    }

    /*
    public void getDiagnosticDataFromVoltageAndGoalRPM()
    {

    }*/
}