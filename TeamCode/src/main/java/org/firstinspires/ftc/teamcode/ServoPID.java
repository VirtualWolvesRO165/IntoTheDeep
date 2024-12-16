package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
@Config

public class ServoPID {

    PIDController pid = new PIDController(0 , 0 , 0);
    public static double kpSv , kdSv , kiSv , ffSv;

    public static double BratSvTarget;

    public double ReturnBratSvTarget(){return BratSvTarget;}

    public void SetBratSvTarget(double x)
    {
        BratSvTarget = x;
    }

    public void PIDServo(CRServo servo , AnalogInput servoPos)
    {
        pid.setPID(kpSv , kiSv , kdSv);
        double pos = servoPos.getVoltage()/3.3*360;
        double pidPow = pid.calculate(pos , BratSvTarget);
        servo.setPower(pidPow + ffSv);
    }

}
