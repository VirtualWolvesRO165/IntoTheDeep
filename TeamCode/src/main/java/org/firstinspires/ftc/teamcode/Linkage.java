package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
@Config
public class Linkage {

    public static double kpBrat = 0.0035, kiBrat = 0, kdBrat = 0, ffBrat = 0.01;
    public static int LinkageTarget = 0 , linkageUpPose=0 , linkageDownPos=2100;

    public static boolean isMoving=false;

    PIDController pidController = new PIDController(0 , 0 , 0);
    public static boolean linkageStatus=false;
    ///false=up true=down
    ///

    public boolean IsMoving()
    {
        return isMoving;
    }

    public void LinkagePID(Gamepad gamepad , DcMotorEx Linkage) {
        if(gamepad.dpad_up && gamepad.left_bumper)
        {
            setBratTarget(linkageUpPose);
            isMoving=true;
        }
        if (gamepad.dpad_down && gamepad.left_bumper)
        {
            setBratTarget(linkageDownPos);
            isMoving=true;
        }
        if(LinkageTarget==linkageUpPose)
            linkageStatus=false;
        else
            linkageStatus=true;
        if(Linkage.getCurrentPosition()==LinkageTarget)
            isMoving=false;

        pidController.setPID(kpBrat, kiBrat, kdBrat);
        int LinkagePos = Linkage.getCurrentPosition();
        double pid = pidController.calculate(LinkagePos, LinkageTarget);
        double pidPowerBRAT = pid + ffBrat;
        Linkage.setPower(pidPowerBRAT);
    }


    public int ReturnLinkageTarget() {
        return LinkageTarget;
    }

    public void setBratTarget(int a) {
        LinkageTarget = a;
    }

    public void ChangeLinkageStatus()
    {
        linkageStatus=!linkageStatus;
    }
    public void SetLinkageStatus(boolean bool){linkageStatus = bool;}

    public boolean ReturnLinkageStatus(){return linkageStatus;}

}
