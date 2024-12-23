package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class BratServo {

    boolean button=false , open=false;
    public static double BratDownPos = 0.51
            , BratUpPose=0.20;

    public double ReturnBratDownPos(){return BratDownPos;}
    public double ReturnBratUpPos(){return BratUpPose;}
    public void BratManager(Gamepad gamepad , Servo BratServoLeft , Servo BratServoRight)
    {
        if(gamepad.right_stick_y>0.1 && (BratServoLeft.getPosition()<=1 ||BratServoLeft.getPosition()>=0 ||BratServoRight.getPosition()<=1 ||BratServoRight.getPosition()>=0))
        {
            setServoPos(BratServoLeft , BratServoRight , BratServoLeft.getPosition()+0.01 , BratServoRight.getPosition()-0.01);
        }
        if(gamepad.right_stick_y<-0.1 && (BratServoLeft.getPosition()<=1 ||BratServoLeft.getPosition()>=0 ||BratServoRight.getPosition()<=1 ||BratServoRight.getPosition()>=0))
            setServoPos(BratServoLeft , BratServoRight , BratServoLeft.getPosition()-0.01 , BratServoRight.getPosition()+0.01);

        if (gamepad.y && !button) {
            open = !open;
            if (open) {
                setServoPos(BratServoLeft , BratServoRight , BratUpPose , BratDownPos);
            } else {
                setServoPos(BratServoLeft , BratServoRight , BratDownPos , BratUpPose);
            }
            button = true;
        }
        if (!gamepad.y) {
            button = false;
        }
    }

    public void setServoPos(Servo servo1 , Servo servo2 , double pos , double pos2) {
        servo1.setPosition(pos);
        servo2.setPosition(pos2);
    }

}
