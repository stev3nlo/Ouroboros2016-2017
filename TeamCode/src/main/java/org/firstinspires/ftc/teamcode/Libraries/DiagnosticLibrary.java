package org.firstinspires.ftc.teamcode.Libraries;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import static java.lang.System.out;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

public class DiagnosticLibrary implements Serializable
{
    HashMap<String,DiagnosticMotorInfo> diagnosticMotorInfos;
    protected static final String fileName = "/sdcard/FIRST/DiagnosticLibraryStorage.txt";

    public DiagnosticLibrary()
    {
        diagnosticMotorInfos = new HashMap<String,DiagnosticMotorInfo>();
        diagnosticMotorInfos.put("motorL1", new DiagnosticMotorInfo("motorL1", new HashMap<Integer,HashMap<Double, DiagnosticData>>()));
        diagnosticMotorInfos.put("motorL2", new DiagnosticMotorInfo("motorL2", new HashMap<Integer,HashMap<Double, DiagnosticData>>()));
        diagnosticMotorInfos.put("motorR1", new DiagnosticMotorInfo("motorR1", new HashMap<Integer,HashMap<Double, DiagnosticData>>()));
        diagnosticMotorInfos.put("motorR2", new DiagnosticMotorInfo("motorR2", new HashMap<Integer,HashMap<Double, DiagnosticData>>()));
        diagnosticMotorInfos.put("motorSpinner", new DiagnosticMotorInfo("motorSpinner", new HashMap<Integer,HashMap<Double, DiagnosticData>>()));

    }
    public void addDiagnosticData(DiagnosticData d)
    {
        getDiagnosticMotorInfoWithMotorName(d.getMotorName()).add(d);
    }

    public DiagnosticMotorInfo getDiagnosticMotorInfoWithMotorName(String motorName)
    {
        out.println("THEDMIS"+diagnosticMotorInfos);
        return diagnosticMotorInfos.get(motorName);
    }

    public double getSpeedForDiagnosticQuery(DiagnosticQuery diagnosticQuery)
    {
        DiagnosticMotorInfo dmi = getDiagnosticMotorInfoWithMotorName(diagnosticQuery.getMotorName());
        out.println("YOLO"+dmi);
        int offset = 0;
        boolean shouldOscillateUp = true;
        double foundPower = -1.0;
        double closestAccuracy = -1;
        while(offset >= -20 && offset <= 20)
        {
            out.println(offset);
            HashMap<Double,DiagnosticData> diagnosticDataByPower = dmi.getHashMapOfDiagnosticDataFromVoltage(diagnosticQuery.getBatteryVoltage()+offset);
            Iterator it = diagnosticDataByPower.entrySet().iterator();
            out.println(diagnosticDataByPower);
            while (it.hasNext()) {
                Map.Entry pair = (Map.Entry)it.next();
                double degreeOfAccuracy = Math.abs(offset) + Math.abs(diagnosticQuery.getGoalRPM()-((DiagnosticData)pair.getValue()).RPM())*50;
                out.println("DOA"+degreeOfAccuracy);
                if(closestAccuracy < 0 || degreeOfAccuracy < closestAccuracy)
                {
                    closestAccuracy = degreeOfAccuracy;
                    foundPower = ((DiagnosticData)pair.getValue()).getPowerSent();
                }
            }
            if(shouldOscillateUp)
            {
                offset *= -1;
                offset += 1;
            }
            else
            {
                offset *= -1;
            }

            shouldOscillateUp = !shouldOscillateUp;
        }
        return foundPower;
    }

    public void saveDiagnosticLibrary() throws IOException
    {
        FileOutputStream fos = new FileOutputStream(fileName);
        ObjectOutputStream ob = new ObjectOutputStream(fos);
        ob.writeObject(this);
        ob.flush();
        ob.close();
    }

    public String toString()
    {
        String s = "";
        Iterator it = diagnosticMotorInfos.entrySet().iterator();
        out.println("TOSTRING"+diagnosticMotorInfos);
        while (it.hasNext())
        {
            Map.Entry pair = (Map.Entry)it.next();
            s += ((DiagnosticMotorInfo)pair.getValue());
        }

        return s;
    }
}