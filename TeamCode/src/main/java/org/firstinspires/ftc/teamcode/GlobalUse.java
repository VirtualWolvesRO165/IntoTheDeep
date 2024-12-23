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
    public static int LiftPickupMax = 2300;
    public static int LiftMaxTarget=4150 , LiftMinTarget=200;
    public static int LiftMultiplier=42;
    public static double kpLift = 0.003, kiLift = 0, kdLift = 0, ffLift = 0.001;

    public static boolean isMoving=false;

    public boolean IsMoving(){return isMoving;}
    public int ReturnLiftTarget(){return LiftTarget;}

    public void SetLiftTarget(int target)
    {
        LiftTarget = target;
    }

    public int ReturnLiftPickUpMax(){return LiftPickupMax;}
    public void LiftControl(Gamepad gamepad , DcMotorEx Lift ,DcMotorEx LiftDreapta, PIDController pid , boolean linkageStatus)
    {
        if(linkageStatus)
        {
            LiftManualControl(gamepad , Lift,LiftDreapta , pid);
        }
        else
            LiftPIDControl(gamepad , Lift,LiftDreapta , pid);
    }

    public void LiftPIDControl(Gamepad gamepad , DcMotor Lift , DcMotorEx LiftDrepata, PIDController liftPID)
    {
        if(LiftTarget<=LiftMaxTarget)
            if (gamepad.left_trigger>0.1){
                LiftTarget+=(int)(gamepad.left_trigger*LiftMultiplier);
                isMoving=true;
            }

        if(LiftTarget>=LiftMinTarget)
            if(gamepad.right_trigger > 0.1){
                LiftTarget-=(int)(gamepad.right_trigger*LiftMultiplier);
                isMoving=true;
            }
        if(LiftTarget>LiftMaxTarget)
            LiftTarget=LiftMaxTarget;
        if(LiftTarget<LiftMinTarget)
            LiftTarget=LiftMinTarget;
        if(Lift.getCurrentPosition()==LiftTarget)
            isMoving=false;
        liftPID.setPID(kpLift , kiLift , kdLift);
        int armPos = Lift.getCurrentPosition();
        double pid = liftPID.calculate(armPos , LiftTarget);
        Lift.setPower(pid+ffLift);
        LiftDrepata.setPower(pid+ffLift);
    }
    public void LiftManualControl(Gamepad gamepad , DcMotorEx Lift , DcMotorEx LiftDreapta,PIDController liftPID)
    {
        if(gamepad.left_trigger>0.1){
            LiftTarget = LiftPickupMax;
            isMoving=true;
        }

        if(gamepad.right_trigger>0.1){
            LiftTarget = LiftMinTarget;
            isMoving=true;
        }

        if(Lift.getCurrentPosition()==LiftTarget)
            isMoving=false;
        liftPID.setPID(kpLift , kiLift , kdLift);
        int armPos = Lift.getCurrentPosition();
        double pid = liftPID.calculate(armPos , LiftTarget);
        Lift.setPower(pid);
        LiftDreapta.setPower(pid);
    }

}
