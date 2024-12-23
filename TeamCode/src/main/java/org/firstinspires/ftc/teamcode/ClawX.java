package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawX {

    boolean open=false , button = false;
    public static double vertical=0 , horizontal=1;

    public double ReturnVerticalPos(){return vertical;}
    public double ReturnHorizontalPos(){return horizontal;}

    public void ClawXManager(Gamepad gamepad , Servo ClawX)
    {
        if (gamepad.b && !button && !gamepad.right_bumper) {
            open = !open;
            if (open) {
                ClawX.setPosition(vertical);
            } else {
                ClawX.setPosition(horizontal);
            }
            button = true;
        }
        if (!gamepad.b && !gamepad.right_bumper) {
            button = false;
        }
    }

    public void setServoPos(Servo servo , double pos)
    {
        servo.setPosition(pos);
    }

    public void ChangeClawState(Servo ClawX)
    {
        if(ClawX.getPosition()==horizontal)
            ClawX.setPosition(vertical);
        else
            ClawX.setPosition(horizontal);
    }
}
