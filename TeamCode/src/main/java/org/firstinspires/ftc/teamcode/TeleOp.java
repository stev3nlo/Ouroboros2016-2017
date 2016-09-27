package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

/**
 * Created by Morgannanez on 9/27/16.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Template: Linear OpMode", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class TeleOp extends MyOpMode {

    //Controller values

    double g1y1;    //left drive
    double g1y2;    //right drive
    double g1x1;
    double g1x2;
    double g2y1;    //lift
    double g2y2;    //manipulator
    double g2x1;
    double g2x2;
    boolean g1Lbump;
    boolean g1Rbump;
    boolean g2Lbump;    //grab cap ball
    boolean g2Rbump;    //release cap ball
    double g1Ltrig;
    double g1Rtrig;
    double g2Ltrig;
    double g2Rtrig;
    boolean g1XPressed;
    boolean g2XPressed;     //beacon presser
    boolean g1YPressed;
    boolean g2YPressed;     //shoot balls
    boolean g1APressed;
    boolean g2APressed;     //drop balls
    boolean g1BPressed;
    boolean g2BPressed;



    public void updateControllerVals()
    {
        g1y1 = gamepad1.left_stick_y;
        g1y2 = gamepad1.right_stick_y;
        g1x1 = gamepad1.left_stick_x;
        g1x2 = gamepad1.right_stick_x;
        g2y1 = gamepad2.left_stick_y;
        g2y2 = gamepad2.right_stick_y;
        g2x1 = gamepad2.left_stick_x;
        g2x2 = gamepad2.right_stick_x;
        g1Lbump = gamepad1.left_bumper;
        g1Rbump = gamepad1.right_bumper;
        g2Lbump = gamepad2.left_bumper;
        g2Rbump = gamepad2.right_bumper;
        g1Ltrig = gamepad1.left_trigger;
        g1Rtrig = gamepad1.right_trigger;
        g2Ltrig = gamepad2.left_trigger;
        g2Rtrig = gamepad2.right_trigger;
        g1XPressed = gamepad1.x;
        g1APressed = gamepad1.a;
        g1YPressed = gamepad1.y;
        g1BPressed = gamepad1.b;
        g2XPressed = gamepad2.x;
        g2APressed = gamepad2.a;
        g2YPressed = gamepad2.y;
        g2BPressed = gamepad2.b;
    }
    public void runOpMode(){
        updateControllerVals();
        while (opModeIsActive()){
            move(g1y2, g1y1); // moves drive wheels




        }
    }

}
