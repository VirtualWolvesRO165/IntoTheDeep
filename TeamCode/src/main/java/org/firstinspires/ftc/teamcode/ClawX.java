package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawX {

    boolean open=false , button = false;

    public int ClawYState=1;

    public void ClawXManager(Gamepad gamepad , Servo ClawX)
    {
        if (gamepad.b && !button && !gamepad.right_bumper) {
            open = !open;
            if (open) {
                ClawX.setPosition(1);
            } else {
                ClawX.setPosition(0.5);
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
        ChangeClawState();
    }

    public void ChangeClawState()
    {
        if(ClawYState==0)
            ClawYState=1;
        else
            ClawYState=0;
    }
}
