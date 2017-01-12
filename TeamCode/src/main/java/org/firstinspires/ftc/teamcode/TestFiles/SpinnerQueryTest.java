package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.DiagnosticLibrary;
import org.firstinspires.ftc.teamcode.Libraries.DiagnosticQuery;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

/**
 * Created by spencersharp on 1/11/17.
 */
@Autonomous(name="Spinner Query Test", group="Test")

public class SpinnerQueryTest extends MyOpMode
{
    DiagnosticLibrary dl;

    public void initializeDiagnosticLibrary()  throws IOException, ClassNotFoundException
    {
        try{
            FileInputStream fis = new FileInputStream("/sdcard/FIRST/DiagnosticLibraryStorage.txt");
            ObjectInputStream ois = new ObjectInputStream(fis);
            //out.println("hello");
            dl = (DiagnosticLibrary) ois.readObject();
            fis.close();
        }
        catch (FileNotFoundException fnes)
        {
            try{
                File file = new File("/sdcard/FIRST/DiagnosticLibraryStorage.txt");
                FileOutputStream fos = new FileOutputStream(file);
                dl = new DiagnosticLibrary();
                ObjectOutputStream oos = new ObjectOutputStream(fos);
                oos.writeObject(dl);
                oos.flush();
                oos.close();
            }
            catch(FileNotFoundException fnfe)
            {

            }
            catch(IOException ioe)
            {

            }
        }
        catch(IOException ioe)
        {

        }
        catch(ClassNotFoundException cnfe)
        {

        }
    }

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        try{initializeDiagnosticLibrary();} catch(IOException ioe){} catch(ClassNotFoundException cnfe){}
        telemetry.addData("initialized",dl);
        telemetry.update();
        waitForStart();
        int testBatteryLevel = 1400;
        while(opModeIsActive() && testBatteryLevel > 1200)
        {
            DiagnosticQuery dq = new DiagnosticQuery("motorSpinner",testBatteryLevel,2.285);
            telemetry.addData("testBatteryLevel",testBatteryLevel);
            telemetry.addData("powerToSend",dl.getSpeedForDiagnosticQuery(dq));
            telemetry.update();
            testBatteryLevel-=10;

            pause(2.0);
            idle();
        }
    }
}
