package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class BratServo {

    boolean button=false , open=false;

    public int ClawYState=1;

    public double BratDownPos = 0.45 , BratUpPose=0.20;
    public void BratManager(Gamepad gamepad , Servo BratServoLeft , Servo BratServoRight)
    {
        if (gamepad.y && !button) {
            open = !open;
            if (open) {
                setServoPos(BratServoLeft , BratDownPos);
                setServoPos(BratServoRight , BratUpPose);
            } else {
                setServoPos(BratServoLeft , BratUpPose);
                setServoPos(BratServoRight , BratDownPos);
            }
            button = true;
        }
        if (!gamepad.y) {
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
