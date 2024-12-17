package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Linkage {

    public static double kpBrat = 0.0035, kiBrat = 0, kdBrat = 0, ffBrat = 0.01;
    public static int LinkageTarget = 0 , linkageUpPose=0 , linkageDownPos=2100;

    PIDController pidController;
    public static boolean linkageStatus=false;
    ///false=up true=down

    public void LinkagePID(Gamepad gamepad , DcMotorEx Linkage) {
        if(gamepad.dpad_up && gamepad.left_bumper)
        {
            setBratTarget(linkageUpPose);
            ChangeLinkageStatus();
        }
        if (gamepad.dpad_down && gamepad.left_bumper)
        {
            setBratTarget(linkageDownPos);
            ChangeLinkageStatus();
        }
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

    public boolean ReturnLinkageStatus(){return linkageStatus;}

}
