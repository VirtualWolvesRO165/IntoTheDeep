package org.firstinspires.ftc.teamcode;

import android.net.LinkAddress;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LinkagePID2 {

    GlobalUse global =  new GlobalUse();
    boolean manualControl = false;
    PIDController pidController = new PIDController(0, 0, 0);

    public DcMotorEx
            MotorLiftDreapta;

    public LinkagePID2(HardwareMap hardwareMap) {
        MotorLiftDreapta = hardwareMap.get(DcMotorEx.class, "LiftDreapta");
        MotorLiftDreapta.setDirection(DcMotorEx.Direction.FORWARD);
        MotorLiftDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorLiftDreapta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MotorLiftDreapta.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setLiftTarget(int n){
        global.SetLiftTarget(n);
    }

    public int ReturnLiftTarget(){return global.ReturnLiftTarget();}

    public void LiftControlRight(Gamepad gamepad)
    {
        //  global.LiftControl(gamepad , MotorLiftDreapta , pidController);
    }

//    public void LinkageControlDreapta(Gamepad gamepad) {
//        if(gamepad.dpad_left)
//        {
//            setLiftTarget(7250);
//            manualControl = false;
//        }
//        else if(gamepad.dpad_right)
//        {
//            setLiftTarget(0);
//            manualControl = false;
//        }
//
//        double manualPower = (gamepad.left_trigger-gamepad.right_trigger+global.getffLift())*0.5;
//
//        if(gamepad.left_trigger > 0.1 || gamepad.right_trigger > 0.1)
//            global.setLiftManualControl(true);
//        if(gamepad.left_trigger > 0.9 || gamepad.right_trigger > 0.9)
//            manualPower = (gamepad.right_stick_y+global.getffLift())*0.7;
//
//        pidController.setPID(global.getKpLift(), global.getKiLift(), global.getKdLift());
//        int LinkagePos = MotorLiftDreapta.getCurrentPosition();
//        double pid = pidController.calculate(LinkagePos, global.ReturnLiftTarget());
//        double pidPowerLinkage = pid + global.getffLift();
//
//        if(manualControl)
//        {
//            MotorLiftDreapta.setPower(manualPower);
//        }
//        else
//        {
//            MotorLiftDreapta.setPower(pidPowerLinkage);
//        }
//    }


}
