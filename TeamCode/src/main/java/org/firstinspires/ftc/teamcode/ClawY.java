package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawY {

    public boolean buttonX=false , ClawOpen=false;
    public int ClawYState=1;
    ///1=pos 1 0= pos 0
    public void ClawYManager(Gamepad gamepad , Servo ClawY)
    {
       if (gamepad.x && !buttonX && !gamepad.right_bumper)
       {
            ClawOpen = !ClawOpen;
            if (ClawOpen)
            {
                setServoPos(ClawY , 1);
            } else
            {
                setServoPos(ClawY , 0);
            }
                buttonX = true;
       }
        if (!gamepad.x && !gamepad.right_bumper) {
            buttonX = false;
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
