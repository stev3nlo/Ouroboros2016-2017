package org.firstinspires.ftc.teamcode.Autonomouses;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRRange;


/**
 * Created by Steven on 11/14/2016.
 */
public class AutoBlueRange extends MyOpMode {
    //travel across the field to far beacon
    //using range1, stop x distance away
    //turn until both range1 and range2 are equal
    //press far beacon
    //back up until at closest beacon
    //press beacon
    //shoot
    SensorMRRange rangeF;
    SensorMRRange rangeB;



    public void runOpMode() throws InterruptedException {
        rangeF.initializeSensors();
        rangeB.initializeSensors();
        super.runOpMode();
        waitForStart();


        //travel across the field to far beacon
        //using range1, stop x distance away
        while(rangeF.getUltraSonicDistance() > 10) //random number!!!! needs to be replaced
        {
          moveForwards(1.0);
        }

        //turn until both range1 and range2 are equal
        while(!(rangeF.getUltraSonicDistance() == rangeB.getUltraSonicDistance()))
        {
            move(0.0, 1.0);
        }

        stopMotors();

        //press far beacon
        pushButton("Blue");

        //back up until at closest beacon, includes stablilizing
        moveBackToWhiteLineODS(1.0);

        //press closest beacon
        pushButton("Blue");

        //runs spinner
        initCurtime();
        double timeAtSpinnerStart = getCurTime();
        while(getCurTime()<timeAtSpinnerStart+1.0)
        {
            initCurtime();
            //shoot();
            idle();
        }

        //drops preplaced balls into spinner
        openServoDropper();
        initCurtime();
        double timeAtBallDrop = getCurTime();
        while(getCurTime()<timeAtBallDrop+5.0)
        {
            initCurtime();
            //shoot();
            idle();
        }
        closeServoDropper();

        //turns off spinnner
        runSpinner(0.0);
    }
}
