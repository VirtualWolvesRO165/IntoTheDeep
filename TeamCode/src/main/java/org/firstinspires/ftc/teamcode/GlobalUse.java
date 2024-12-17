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



    public int ReturnLiftTarget(){return LiftTarget;}
    public void SetLiftTarget(int target)
    {
        LiftTarget = target;
    }


    public void LiftControl(Gamepad gamepad , DcMotorEx MotorLiftStanga , DcMotorEx MotorLiftDrapta , boolean linkageStatus)
    {
        if(linkageStatus)
        {
                LiftPickupControl(gamepad , MotorLiftStanga , MotorLiftDrapta);
        }
        else
            LiftDropControl(gamepad , MotorLiftStanga , MotorLiftDrapta);
    }

    public void LiftDropControl(Gamepad gamepad , DcMotor MotorLiftStanga , DcMotorEx MotorLiftDreapta)
    {
        if(LiftTarget<=LiftMaxTarget)
            if (gamepad.left_trigger > 0.1 && gamepad.right_trigger<0.1)
                LiftTarget+=(int)(gamepad.left_trigger*LiftMultiplier);
        if(LiftTarget>=200)
            if(gamepad.right_trigger > 0.1 && gamepad.left_trigger<0.1)
                LiftTarget-=(int)(gamepad.right_trigger*LiftMultiplier);
        MotorLiftStanga.setTargetPosition(LiftTarget);
        if(MotorLiftStanga.getTargetPosition()!=LiftTarget)
        {
            MotorLiftStanga.setPower(0.5);
            MotorLiftDreapta.setPower(0.5);
        }
        else
        {
            MotorLiftStanga.setPower(0);
            MotorLiftDreapta.setPower(0);
        }
    }
    public void LiftPickupControl(Gamepad gamepad , DcMotorEx MotorLiftStanga , DcMotorEx MotorLiftDreapta)
    {
        if(gamepad.left_trigger>0.1)
            LiftTarget = LiftPickupMax;
        if(gamepad.right_trigger>0.1)
            LiftTarget = LiftMinTarget;

        MotorLiftStanga.setTargetPosition(LiftTarget);
        if(MotorLiftStanga.getTargetPosition()!=LiftTarget)
        {
            MotorLiftStanga.setPower(0.5);
            MotorLiftDreapta.setPower(0.5);
        }
        else
        {
            MotorLiftStanga.setPower(0);
            MotorLiftDreapta.setPower(0);
        }
    }

}
