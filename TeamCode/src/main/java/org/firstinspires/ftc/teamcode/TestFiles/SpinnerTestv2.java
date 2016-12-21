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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Spinner Test v2", group="Test")
public class SpinnerTestv2 extends TeleOp
{
    public void initialize(){
        motorSpinner = hardwareMap.dcMotor.get("motorSpinner");
        curPowerOfMotorSpinner = 0.3;
    }
    public void runOpMode()
    {
        initialize();
        waitForStart();
        while(opModeIsActive()) {
            initCurtime();
            updateControllerVals();
            telemetry.addData("curSpeed",curPowerOfMotorSpinner);
            if (g1YPressed || !shooterIsRunning) {
                runSpinner(0.0);
                shooterIsRunning = false;
                firstCycleOfSpinner = true;
                telemetry.addData("stabilzing","false");
                //firstCycleOfSpinner = true;
            }
            //creates constant speed of spinner throughout game
            if (g1XPressed || shooterIsRunning) {
                shooterIsRunning = true;
                if(firstCycleOfSpinner) {
                    firstCycleOfSpinner = false;
                    initTime = getCurTime();
                }
                runRPMStabilization();
                runSpinner(curPowerOfMotorSpinner);
            }
            telemetry.addData("curRPM", curRPM);
            telemetry.update();
            idle();
        }
    }
}
