package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.DiagnosticData;
import org.firstinspires.ftc.teamcode.Libraries.DiagnosticLibrary;
import org.firstinspires.ftc.teamcode.Libraries.DiagnosticQuery;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;

/**
 * Created by spencersharp on 1/7/17.
 */
@TeleOp(name = "DiagnosticTeleopTest", group = "Teleop")

public class DiagnosticTeleopTest extends MyOpMode
{
    DiagnosticLibrary dl;
    double motorL1MaxRPM;
    double motorL2MaxRPM;
    double motorR1MaxRPM;
    double motorR2MaxRPM;
    double motorSpinnerMaxRPM;
    public void initializeDiagnosticLibrary()  throws IOException, ClassNotFoundException
    {
        try{
            FileInputStream fis = new FileInputStream("/sdcard/FIRST/DiagnosticLibraryStorage.txt");
            ObjectInputStream ois = new ObjectInputStream(fis);
            dl = (DiagnosticLibrary) ois.readObject();
            fis.close();
        }
        catch (FileNotFoundException fnes)
        {
            File file = new File("/sdcard/FIRST/DiagnosticLibraryStorage.txt");
            FileOutputStream fos = new FileOutputStream(file);
            fos.flush();
            fos.close();
            dl = new DiagnosticLibrary();
        }
        catch(IOException ioe)
        {

        }
        catch(ClassNotFoundException cnfe)
        {

        }
    }

    public void initializeMaxRPMs()
    {

    }
    public void runOpMode() throws InterruptedException
    {
        try{
            initializeDiagnosticLibrary();
        }
        catch(Exception e)
        {
        }
        while(opModeIsActive())
        {
            double batteryVoltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
            //DiagnosticQuery dqmotorL1 = new DiagnosticQuery("motorL1",batteryVoltage,)
            move(gamepad1.left_stick_y, (-gamepad1.right_stick_y)/1.2);
            if(gamepad1.b)
            {
            }
        }
    }
}
