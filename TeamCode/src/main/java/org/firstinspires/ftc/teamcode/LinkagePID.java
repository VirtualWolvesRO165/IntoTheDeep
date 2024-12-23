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
public class LinkagePID {
    public static double kpLinkage = 0.0035, kiLinkage = 0, kdLinkage = 0, ffLinkage = 0.01;
    GlobalUse global = new GlobalUse();
    boolean manualControl = false;
    PIDController pidController = new PIDController(0, 0, 0);

    public DcMotorEx
            MotorLiftStanga;

    public LinkagePID(HardwareMap hardwareMap) {
        MotorLiftStanga = hardwareMap.get(DcMotorEx.class, "LiftStanga");
        MotorLiftStanga.setDirection(DcMotorEx.Direction.REVERSE);
        MotorLiftStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorLiftStanga.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MotorLiftStanga.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setLiftTarget(int n){
        global.SetLiftTarget(n);
    }

    public int ReturnLiftTarget(){return global.ReturnLiftTarget();}

    public void LiftPIDControlStanga(Gamepad gamepad)
    {
        // global.LiftControl(gamepad , MotorLiftStanga , pidController);
    }



}
