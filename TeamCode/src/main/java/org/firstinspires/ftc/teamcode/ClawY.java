package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawY {

    public boolean buttonX=false , ClawOpen=false;
    public static double dropPos=1 , pickupPos=0;

    public double ReturnDropPos(){return dropPos;}
    public double ReturnPickupPos(){return pickupPos;}
    public void ClawYManager(Gamepad gamepad , Servo ClawY)
    {
        if (gamepad.x && !buttonX && !gamepad.right_bumper)
        {
            ClawOpen = !ClawOpen;
            if (ClawOpen)
            {
                ChangeClawState(ClawY);
            } else
            {
                ChangeClawState(ClawY);
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
    }

    public void ChangeClawState(Servo ClawY)
    {
        if(ClawY.getPosition()==dropPos)
            ClawY.setPosition(pickupPos);
        else
            ClawY.setPosition(dropPos);
    }

}
