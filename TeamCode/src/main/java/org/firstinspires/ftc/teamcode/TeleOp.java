package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorAdafruitIMU;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Teleop", group="Teleop")  // @MyAutonomous(...) is the other common choice
public class TeleOp extends MyOpMode
{
    public boolean shooterIsRunning = false;
    //Controller values

    public double g1y1;    //left drive
    public double g1y2;    //right drive
    public double g1x1;
    public double g1x2;
    public boolean g1Lbump;
    public boolean g1Rbump;
    public double g1Ltrig;
    public double g1Rtrig;
    public boolean g1XPressed;
    public boolean g1YPressed;
    public boolean g1APressed;
    public boolean g1BPressed;

    public double curYaw;
    public String beaconColor;

    int numLoopsRemainining;
    double timeAtLastLoop;
    double timePerLoop = 1.0;
    public int driverModeLeft = 1;
    public int driverModeRight = 1;

    public double g2y1;    //lift
    public double g2y2;    //manipulator
    public double g2x1;
    public double g2x2;
    public boolean g2Lbump;    //grab cap ball
    public boolean g2Rbump;    //release cap ball
    public double g2Ltrig;
    public double g2Rtrig;
    public boolean g2XPressed;     //start shooter spinner
    public boolean g2YPressed;     //stop shooter spinner
    public boolean g2APressed;     //toggle servo dropper
    public boolean g2BPressed;

    long timeAtEndOfLastCycle;

    double timeBallsFinishDropping;

    public boolean firstCycleOfSpinner;

    boolean isInterruptibleRoutineRedToBlueRunning = false;
    boolean isInterruptibleRoutineBlueToRedRunning = false;
    boolean hasGyroInitValueAndEncoderValueForInterruptibleRoutineBlueToRedOrRedToBlueBeenInitialized = false;
    boolean hasInterruptibleRoutineBeganTurning = false;
    boolean hasInterruptibleRoutineBeganRollingAlongWall = false;
    double gyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlue = 0.0;

    boolean isInterruptibleRoutineDriveForwardsToRedAndPressRunning = false;
    boolean isInterruptibleRoutineDriveBackwardsToRedAndPressRunning = false;
    boolean isInterruptibleRoutineDriveForwardsToBlueAndPressRunning = false;
    boolean isInterruptibleRoutineDriveBackwardsToBlueAndPressRunning = false;
    boolean inDrivingRoutineHasColorBeenSeen = false;

    int interruptibleRoutineInitEncoderVal = 0;

    double startTimeInterruptibleRoutineRedToBlue = 0.0;
    double startTimeInterruptibleRoutineBlueToRed = 0.0;

    boolean areRollersRaised = true;

    public void updateControllerVals()
    {
        g1y1 = gamepad1.left_stick_y;
        g1y2 = gamepad1.right_stick_y;
        g1x1 = gamepad1.left_stick_x;
        g1x2 = gamepad1.right_stick_x;
        g1Lbump = gamepad1.left_bumper;
        g1Rbump = gamepad1.right_bumper;
        g1Ltrig = gamepad1.left_trigger;
        g1Rtrig = gamepad1.right_trigger;
        g1XPressed = gamepad1.x;
        g1APressed = gamepad1.a;
        g1YPressed = gamepad1.y;
        g1BPressed = gamepad1.b;

        g2y1 = gamepad2.left_stick_y;
        g2y2 = gamepad2.right_stick_y;
        g2x1 = gamepad2.left_stick_x;
        g2x2 = gamepad2.right_stick_x;
        g2Lbump = gamepad2.left_bumper;
        g2Rbump = gamepad2.right_bumper;
        g2Ltrig = gamepad2.left_trigger;
        g2Rtrig = gamepad2.right_trigger;
        g2XPressed = gamepad2.x;
        g2APressed = gamepad2.a;    //toggle servo dropper
        g2YPressed = gamepad2.y;
        g2BPressed = gamepad2.b;
    }

    public void updateTelemetry()
    {
        telemetry.addData("BEACON PUSH INFO","");
        telemetry.addData("curPowerOfMotorSpinner",curPowerOfMotorSpinner);
        if(g2Ltrig > 0.6)
        {
            telemetry.addData("g2Ltrig being pressed", "LEFT");
        }
        else if(g2Rtrig > 0.6)
        {
            telemetry.addData("g2Rtrig being pressed", "RIGHT");
        }
        else if(g2Rbump || g2Lbump)
        {
            telemetry.addData("g2Rbump or g2Lbump being pressed", "MIDDLE");
        }
        else
        {
            telemetry.addData("No beacon control being pressed","");
        }
        if(g1Ltrig > 0.6)
        {
            telemetry.addData("g1LTrig","PRESSED");
        }
        else
        {
            telemetry.addData("g1LTrig","NOT PRESSED");
        }
        if(g1Rtrig > 0.6)
        {
            telemetry.addData("g1RTrig","PRESSED");
        }
        else
        {
            telemetry.addData("g1RTrig","NOT PRESSED");
        }
        telemetry.update();
    }

    public void initializeBeaconColorSensor()
    {
        colorB = new SensorMRColor(hardwareMap.colorSensor.get("colorB"));
        colorB.sensorSetup(0x2c);
    }

    public void initializeGyro()
    {
        gyro = new SensorAdafruitIMU(hardwareMap.get(BNO055IMU.class, "gyro"));
        telemetry.addData("Gyro", "Initialized");
        telemetry.update();
    }

    public boolean isInterruptibleRoutineRunning()
    {
        return isInterruptibleRoutineBlueToRedRunning || isInterruptibleRoutineRedToBlueRunning || isInterruptibleRoutineDriveBackwardsToBlueAndPressRunning || isInterruptibleRoutineDriveForwardsToBlueAndPressRunning || isInterruptibleRoutineDriveForwardsToRedAndPressRunning || isInterruptibleRoutineDriveBackwardsToRedAndPressRunning;
    }

    public void stopAllRoutines()
    {
        isInterruptibleRoutineRedToBlueRunning = false;
        isInterruptibleRoutineBlueToRedRunning = false;
        isInterruptibleRoutineDriveForwardsToRedAndPressRunning = false;
        isInterruptibleRoutineDriveBackwardsToRedAndPressRunning = false;
        isInterruptibleRoutineDriveForwardsToBlueAndPressRunning = false;
        isInterruptibleRoutineDriveBackwardsToBlueAndPressRunning = false;
    }

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        initializeBeaconColorSensor();
        initializeGyro();
        beaconColor = colorB.beaconColor();
        curYaw = gyro.getYaw();
        waitForStart();

        timeAtLastLoop = getCurTime();
        while (opModeIsActive())
        {
            updateControllerVals();
            initCurtime(); //gets real time timer

            if(g1Ltrig > 0.6)
                driverModeLeft = 3;
            else if(g1Lbump)
                driverModeLeft = 2;
            else
                driverModeLeft = 1;

            if(g1Rtrig > 0.6)
                driverModeRight = 3;
            else if(g1Rbump)
                driverModeRight = 2;
            else
                driverModeRight = 1;
            telemetry.addData("driverModeLeft",driverModeLeft);
            telemetry.addData("driverModeRight",driverModeRight);

            if(g2y1>0.1)
                moveManip(1.0);
            else if(g2y1 < -0.1)
                moveManip(-1.0);
            else
                moveManip(0.0);

            //Stabilization

            if (g2YPressed || !shooterIsRunning)
            {
                shooterIsRunning = false;
                runSpinner(0.0);
            }
            //creates constant speed of spinner throughout game
            if(g2XPressed || shooterIsRunning)
            {
                shooterIsRunning = true;
                runSpinner(0.86);
            }
            // stops spinner

            //releases balls from basket into spinner
            if (g2APressed)// && shooterIsRunning)
            {
                openServoDropper();
            }
            else if(g2BPressed)// || !shooterIsRunning)
            {
                closeServoDropper();
            }

            if(g1BPressed) //red
            {
                if(gamepad1.dpad_up)
                {
                    stopAllRoutines();
                    isInterruptibleRoutineDriveForwardsToRedAndPressRunning = true;
                }
                else if(gamepad1.dpad_down)
                {
                    stopAllRoutines();
                    isInterruptibleRoutineDriveBackwardsToRedAndPressRunning = true;
                }
            }
            else if(g1XPressed) //blue
            {
                if(gamepad1.dpad_up)
                {
                    stopAllRoutines();
                    isInterruptibleRoutineDriveForwardsToBlueAndPressRunning = true;
                }
                else if(gamepad1.dpad_down)
                {
                    stopAllRoutines();
                    isInterruptibleRoutineDriveBackwardsToBlueAndPressRunning = true;
                }
            }
            else if(!isInterruptibleRoutineBlueToRedRunning && g1YPressed) //blue to red (forwards)
            {
                stopAllRoutines();
                isInterruptibleRoutineBlueToRedRunning = true;
                hasInterruptibleRoutineBeganTurning = false;
                hasGyroInitValueAndEncoderValueForInterruptibleRoutineBlueToRedOrRedToBlueBeenInitialized = false;
                hasInterruptibleRoutineBeganRollingAlongWall = false;
                startTimeInterruptibleRoutineBlueToRed = getCurTime();
            }
            else if(!isInterruptibleRoutineRedToBlueRunning && g1APressed && !gamepad1.start) //red to blue (backwards)
            {
                stopAllRoutines();
                isInterruptibleRoutineRedToBlueRunning = true;
                hasInterruptibleRoutineBeganTurning = false;
                hasGyroInitValueAndEncoderValueForInterruptibleRoutineBlueToRedOrRedToBlueBeenInitialized = false;
                hasInterruptibleRoutineBeganRollingAlongWall = false;
                startTimeInterruptibleRoutineRedToBlue = getCurTime();
            }


            int interruptibleRoutineRedToBlueDist = 1200;
            int interruptibleRoutineBlueToRedDist = 1500;

            if(areRollersDropping && getCurTime() - rollerMovementTimeDown > startTimeOfDroppingRollers)
            {
                stopRollers();
                areRollersDropping = false;
                areRollersRaised = false;
            }
            else if(areRollersRaising && getCurTime() - rollerMovementTimeUp > startTimeOfRaisingRollers)
            {
                holdRollersUp();
                areRollersRaising = false;
                areRollersRaised = true;
            }

            if(isInterruptibleRoutineRunning()) {
                curYaw = gyro.getYaw();
                beaconColor = colorB.beaconColor();
                telemetry.addData("yaw", curYaw);
                telemetry.addData("beaconColor", beaconColor);
            }
            if(isInterruptibleRoutineDriveForwardsToRedAndPressRunning)
            {
                telemetry.addData("isInterruptibleRoutineDriveForwardsToRedAndPressRunning",true);
                if(g1y1>0.1 || g1y1<-0.1 || g1y2 > 0.1 || g1y2 < -0.1) {
                    isInterruptibleRoutineDriveForwardsToRedAndPressRunning = false;
                    telemetry.addData("BROKEN", "ROUTINE");
                    moveForwards(0.0);
                }
                else {
                    moveForwards(0.34, 0.2);
                    if (beaconColor.equals("Red")) {
                        moveBackwards(0.03);
                        isInterruptibleRoutineDriveForwardsToRedAndPressRunning = false;
                        pushButtonWithRollersQuick();
                    }
                }
            }
            else if(isInterruptibleRoutineDriveBackwardsToRedAndPressRunning)
            {
                telemetry.addData("isInterruptibleRoutineDriveForwardsToBlueAndPressRunning",true);
                if(g1y1>0.1 || g1y1<-0.1 || g1y2 > 0.1 || g1y2 < -0.1) {
                    isInterruptibleRoutineDriveBackwardsToRedAndPressRunning = false;
                    telemetry.addData("BROKEN","ROUTINE");
                    moveForwards(0.0);
                }
                else {
                    moveBackwards(0.34, 0.2);
                    if (beaconColor.equals("Red")) {
                        moveForwards(0.03);
                        isInterruptibleRoutineDriveBackwardsToRedAndPressRunning = false;
                        pushButtonWithRollersQuick();
                    }
                }
            }
            else if(isInterruptibleRoutineDriveForwardsToBlueAndPressRunning)
            {
                telemetry.addData("isInterruptibleRoutineDriveForwardsToBlueAndPressRunning",true);
                if(g1y1>0.1 || g1y1<-0.1 || g1y2 > 0.1 || g1y2 < -0.1) {
                    isInterruptibleRoutineDriveForwardsToBlueAndPressRunning = false;
                    telemetry.addData("BROKEN","ROUTINE");
                    moveForwards(0.0);
                }
                else {
                    moveForwards(0.34, 0.2);
                    if (beaconColor.equals("Blue")) {
                        moveBackwards(0.03);
                        isInterruptibleRoutineDriveForwardsToBlueAndPressRunning = false;
                        pushButtonWithRollersQuick();
                    }
                }
            }
            else if(isInterruptibleRoutineDriveBackwardsToBlueAndPressRunning)
            {
                telemetry.addData("isInterruptibleRoutineDriveBackwardsToBlueAndPressRunning",true);
                if(g1y1>0.1 || g1y1<-0.1 || g1y2 > 0.1 || g1y2 < -0.1) {
                    isInterruptibleRoutineDriveBackwardsToBlueAndPressRunning = false;
                    telemetry.addData("BROKEN","ROUTINE");
                    moveForwards(0.0);
                }
                else {
                    moveBackwards(0.34, 0.2);
                    if (beaconColor.equals("Blue")) {
                        moveForwards(0.03);
                        isInterruptibleRoutineDriveBackwardsToBlueAndPressRunning = false;
                        pushButtonWithRollersQuick();
                    }
                }
            }
            else if(isInterruptibleRoutineBlueToRedRunning)
            {
                if(g1y1>0.1 || g1y1<-0.1 || g1y2 > 0.1 || g1y2 < -0.1) {
                    isInterruptibleRoutineBlueToRedRunning = false;
                    telemetry.addData("BROKEN","ROUTINE");
                    moveForwards(0.0);
                }
                else {
                    telemetry.addData("interruptibleRoutineRunning", "true");
                    if (!hasGyroInitValueAndEncoderValueForInterruptibleRoutineBlueToRedOrRedToBlueBeenInitialized) {
                        gyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlue = gyro.getYaw();
                        interruptibleRoutineInitEncoderVal = getAvgEnc();
                        hasGyroInitValueAndEncoderValueForInterruptibleRoutineBlueToRedOrRedToBlueBeenInitialized = true;
                    }
                    int curDist = getAvgEnc();
                    if (hasInterruptibleRoutineBeganRollingAlongWall) {
                        moveForwards(0.34, 0.2);
                        telemetry.addData("driving along next wall",true);
                    }
                    else if(hasInterruptibleRoutineBeganTurning)
                    {
                        if(getAngleDiff(gyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlue, curYaw) >= 65)
                        {
                            hasInterruptibleRoutineBeganTurning = false;
                            hasInterruptibleRoutineBeganRollingAlongWall = true;
                            moveForwards(0.34, 0.2);
                            telemetry.addData("driving along next wall",true);
                        }
                        else {
                            moveForwards(0.0, 0.5);

                            telemetry.addData("hard turn", true);
                        }
                    }
                    else if (Math.abs(curDist-interruptibleRoutineInitEncoderVal) < interruptibleRoutineBlueToRedDist) {
                        moveForwards(0.25, 0.25);
                        telemetry.addData("initial straight",true);
                    } else if (Math.abs(curDist-interruptibleRoutineInitEncoderVal) >= interruptibleRoutineBlueToRedDist) {
                        hasInterruptibleRoutineBeganTurning = true;
                        moveForwards(0.0, 0.5);
                        telemetry.addData("hard turn",true);
                    }
                    telemetry.addData("timeRunningIt",getCurTime()-startTimeInterruptibleRoutineBlueToRed);

                    if (getCurTime() - startTimeInterruptibleRoutineRedToBlue > 1.0 && !beaconColor.equals("Neither")) {
                        if ((g1BPressed && beaconColor.equals("Red")) || (g1XPressed && beaconColor.equals("Blue"))) {
                            isInterruptibleRoutineBlueToRedRunning = false;
                            hasInterruptibleRoutineBeganRollingAlongWall = false;
                            hasInterruptibleRoutineBeganTurning = false;
                            moveBackwards(0.05);
                            pushButtonWithRollersQuick();
                        } else if (!g1BPressed && !g1XPressed) {
                            isInterruptibleRoutineBlueToRedRunning = false;
                            hasInterruptibleRoutineBeganRollingAlongWall = false;
                            hasInterruptibleRoutineBeganTurning = false;
                            moveBackwards(0.05);
                        }
                    }
                }
            }
            else if(isInterruptibleRoutineRedToBlueRunning)
            {
                if(g1y1>0.1 || g1y1<-0.1 || g1y2 > 0.1 || g1y2 < -0.1) {
                    isInterruptibleRoutineRedToBlueRunning = false;
                    telemetry.addData("BROKEN","ROUTINE");
                    moveForwards(0.0);
                }
                else {
                    telemetry.addData("interruptibleRoutineRunning", "true");
                    if (!hasGyroInitValueAndEncoderValueForInterruptibleRoutineBlueToRedOrRedToBlueBeenInitialized) {
                        gyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlue = gyro.getYaw();
                        interruptibleRoutineInitEncoderVal = getAvgEnc();
                        hasGyroInitValueAndEncoderValueForInterruptibleRoutineBlueToRedOrRedToBlueBeenInitialized = true;
                    }
                    int curDist = getAvgEnc();
                    if (hasInterruptibleRoutineBeganRollingAlongWall) {
                        moveForwards(0.34, 0.2);
                        telemetry.addData("driving along next wall",true);
                    }
                    else if(hasInterruptibleRoutineBeganTurning)
                    {
                        if(getAngleDiff(gyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlue, curYaw) >= 65)
                        {
                            hasInterruptibleRoutineBeganTurning = false;
                            hasInterruptibleRoutineBeganRollingAlongWall = true;
                            moveBackwards(0.34, 0.2);
                            telemetry.addData("driving along next wall",true);
                        }
                        else {
                            moveBackwards(0.0, 0.5);

                            telemetry.addData("hard turn", true);
                        }
                    }
                    else if (Math.abs(curDist-interruptibleRoutineInitEncoderVal) < interruptibleRoutineRedToBlueDist) {
                        moveBackwards(0.25, 0.25);
                        telemetry.addData("initial straight",true);
                    } else if (Math.abs(curDist-interruptibleRoutineInitEncoderVal) >= interruptibleRoutineRedToBlueDist) {
                        hasInterruptibleRoutineBeganTurning = true;
                        moveBackwards(0.0, 0.5);
                        telemetry.addData("hard turn",true);
                    }
                    telemetry.addData("timeRunningIt", getCurTime() - startTimeInterruptibleRoutineBlueToRed);

                    if (getCurTime() - startTimeInterruptibleRoutineRedToBlue > 1.0 && !beaconColor.equals("Neither")) {
                        if ((g1BPressed && beaconColor.equals("Red")) || (g1XPressed && beaconColor.equals("Blue"))) {
                            isInterruptibleRoutineRedToBlueRunning = false;
                            hasInterruptibleRoutineBeganRollingAlongWall = false;
                            hasInterruptibleRoutineBeganTurning = false;
                            moveForwards(0.05);
                            pushButtonWithRollersQuick();
                            moveForwards(0.0);
                        } else if (!g1BPressed && !g1XPressed) {
                            isInterruptibleRoutineRedToBlueRunning = false;
                            hasInterruptibleRoutineBeganRollingAlongWall = false;
                            hasInterruptibleRoutineBeganTurning = false;
                            moveForwards(0.05);
                        }
                    }
                }
            }

            if(g2Ltrig > 0.6) //drop rollers
            {
                if(!areRollersDropping)
                {
                    if(areRollersRaising)
                    {
                        startTimeOfDroppingRollers = getCurTime() + getCurTime() - rollerMovementTimeDown - startTimeOfRaisingRollers;
                        areRollersRaising = false;
                    }
                    else
                    {
                        startTimeOfDroppingRollers = getCurTime();
                    }
                    moveRollersDown();
                    areRollersDropping = true;
                }
            }
            else if(g2Lbump) //raise rollers
            {
                if(!areRollersRaising)
                {
                    if(areRollersDropping)
                    {
                        startTimeOfRaisingRollers = getCurTime() + getCurTime() - rollerMovementTimeUp - startTimeOfDroppingRollers;
                        areRollersDropping = false;
                    }
                    else
                    {
                        startTimeOfRaisingRollers = getCurTime();
                    }
                    moveRollersUp();
                    areRollersRaising = true;
                }
            }
            else if(g2Rtrig > 0.6)
            {
                moveBeaconPusherOut();
            }
            else if(g2Rbump) {
                moveBeaconPusherIn();
            }

            if(!isInterruptibleRoutineRunning()) {
                if (g1y1 > 0.1 || g1y1 < -0.1) {
                    motorL1.setPower(g1y1 / driverModeLeft);
                    motorL2.setPower(g1y1 / driverModeLeft);
                } else {
                    motorL1.setPower(0.0);
                    motorL2.setPower(0.0);
                }
                if (g1y2 > 0.1 || g1y2 < -0.1) {
                    motorR1.setPower(-g1y2 / driverModeRight);
                    motorR2.setPower(-g1y2 / driverModeRight);
                } else {
                    motorR1.setPower(0.0);
                    motorR2.setPower(0.0);
                }
            }

            updateTelemetry();
            idle();
        }
    }
}