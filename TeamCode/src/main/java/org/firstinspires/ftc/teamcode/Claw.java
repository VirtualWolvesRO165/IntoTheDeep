package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    boolean button=false;
    public static boolean clawState=false;
    ///false=closed true=opened
    public static double openPos=1 , closePos=0;

    public double ReturnOpenPos(){return openPos;}
    public double ReturnClosePos(){return closePos;}

    public void SetClawState(boolean state)
    {
        clawState = state;
    }
    public boolean ReturnClawState(){return clawState;}
    public void ClawManager(Gamepad gamepad , Servo Claw)
    {
        ClawState(Claw);
    }
    public void ClawState(Servo Claw)
    {
        if(clawState)
            Claw.setPosition(openPos);
        else
            Claw.setPosition(closePos);
    }

}
