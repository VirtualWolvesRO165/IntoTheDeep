package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    boolean button=false , open=false;

    public int ClawYState=1;
    public void ClawManager(Gamepad gamepad , Servo Claw)
    {
        if (gamepad.a && !button) {
            open = !open;
            if (open) {
                setServoPos(Claw , 1);
            } else {
                setServoPos(Claw , 0);
            }
            button = true;
        }
        if (!gamepad.a) {
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
