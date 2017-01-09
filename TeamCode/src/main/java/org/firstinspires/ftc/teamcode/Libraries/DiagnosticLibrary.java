package org.firstinspires.ftc.teamcode.Libraries;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutput;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashSet;

/**
 * Created by spencersharp on 1/7/17.
 */
public class DiagnosticLibrary implements Serializable
{
    HashSet<DiagnosticMotorInfo> diagnosticMotorInfos;
    protected static final String fileName = "/sdcard/FIRST/DiagnosticLibraryStorage.txt";

    public DiagnosticLibrary()
    {
        diagnosticMotorInfos = new HashSet<DiagnosticMotorInfo>();

    }
    public void addDiagnosticData(DiagnosticData d)
    {
        getDiagnosticMotorInfoWithMotorName(d.getMotorName()).add(d);
    }

    public DiagnosticMotorInfo getDiagnosticMotorInfoWithMotorName(String motorName)
    {
        for(DiagnosticMotorInfo dmi : diagnosticMotorInfos)
            if(dmi.getMotorName().equals(motorName))
                return dmi;
        return null;
    }

    public double getSpeedForDiagnosticQuery(DiagnosticQuery diagnosticQuery)
    {
        return 0.0;
    }

    public void saveDiagnosticLibrary() throws IOException
    {
        FileOutputStream fos = new FileOutputStream(fileName);
        ObjectOutputStream ob = new ObjectOutputStream(fos);
        ob.writeObject(this);
        ob.flush();
    }
}