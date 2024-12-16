package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config

public class GlobalUse {

    ///Lift
    public static int LiftTarget;
    public static int LiftPickupMax = 3000;
    public static int LiftMaxTarget=7250 , LiftMinTarget=200;
    public static int LiftMultiplier=42;
    public static boolean LiftManualControl=false;
    public static double kpLift = 0.003, kiLift = 0, kdLift = 0, ffLift = 0.001;

    boolean manualControl = false;
    //LiftPosition
   // int LiftDownPosition=0, LiftUpPosition=500;

    //Linkage
    int linkageDownPos =2100, linkageUpPose=0;

    public static boolean linkageStatus = false;
    //false=up true=down;

    public void ChangeLinkageStatus(boolean bool)
    {
        linkageStatus = bool;
    }

    public boolean ReturnLinkageStatus()
    {
        return linkageStatus;
    }
    public int ReturnLiftTarget(){return LiftTarget;}

    public void SetLiftTarget(int target)
    {
        LiftTarget = target;
    }


    public void LiftControl(Gamepad gamepad , DcMotorEx Lift , PIDController pid)
    {
        if(linkageStatus)
        {
                LiftManualControl(gamepad , Lift , pid);
        }
        else
            LiftPIDControl(gamepad , Lift , pid);
    }

    public void LiftPIDControl(Gamepad gamepad , DcMotor Lift , PIDController liftPID)
    {
//        if(gamepad.dpad_left)
//            LiftTarget = LiftMaxTarget;
//        if(gamepad.dpad_right)
//            LiftTarget = LiftMinTarget;
//        liftPID.setPID(kpLift , kiLift , kdLift);
//        int armPos = Lift.getCurrentPosition();
//        double pid = liftPID.calculate(armPos , LiftTarget);
//        Lift.setPower(pid);

        if(LiftTarget<=LiftMaxTarget)
            if (gamepad.left_trigger > 0.1 && gamepad.right_trigger<0.1)
                LiftTarget+=(int)(gamepad.left_trigger*LiftMultiplier);
        if(LiftTarget>=200)
            if(gamepad.right_trigger > 0.1 && gamepad.left_trigger<0.1)
                LiftTarget-=(int)(gamepad.right_trigger*LiftMultiplier);
        liftPID.setPID(kpLift , kiLift , kdLift);
        int armPos = Lift.getCurrentPosition();
        double pid = liftPID.calculate(armPos , LiftTarget);
        Lift.setPower(pid);
    }
    public void LiftManualControl(Gamepad gamepad , DcMotorEx Lift , PIDController liftPID)
    {
//        double manualPower = (gamepad.left_trigger-gamepad.right_trigger)*0.5;
//
//
//        if (gamepad.left_trigger > 0.1 || gamepad.right_trigger > 0.1)
//            manualControl = true;
//        if (gamepad.left_trigger > 0.9 || gamepad.right_trigger > 0.9)
//            manualPower = (gamepad.left_trigger - gamepad.right_trigger) * 0.7;
//
//        liftPID.setPID(kpLift, kiLift, kdLift);
//        int armPos = Lift.getCurrentPosition();
//        double pid = liftPID.calculate(armPos, LiftTarget);
//
//        if(manualControl)
//        {
//            Lift.setPower(manualPower);
//        }
//        else
//        {
//            Lift.setPower(pid);
//        }

//        if(LiftTarget<=5000)
//            if (gamepad.left_trigger > 0.1 && gamepad.right_trigger<0.1)
//               LiftTarget+=(int)(gamepad.left_trigger*LiftMultiplier);
//        if(LiftTarget>=200)
//            if(gamepad.right_trigger > 0.1 && gamepad.left_trigger<0.1)
//              LiftTarget-=(int)(gamepad.right_trigger*LiftMultiplier);

        if(gamepad.left_trigger>0.1)
            LiftTarget = LiftPickupMax;
        if(gamepad.right_trigger>0.1)
            LiftTarget = LiftMinTarget;
        liftPID.setPID(kpLift , kiLift , kdLift);
        int armPos = Lift.getCurrentPosition();
        double pid = liftPID.calculate(armPos , LiftTarget);
        Lift.setPower(pid);
    }

}
