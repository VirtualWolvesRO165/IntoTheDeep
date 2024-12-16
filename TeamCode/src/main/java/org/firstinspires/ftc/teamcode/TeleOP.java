package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.videoio.VideoCapture;

import java.util.List;

@TeleOp(name="1TELEOP")
public class TeleOP extends LinearOpMode {
    double x;
    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        RobotHardware robot = new RobotHardware(hardwareMap);
        LinkagePID lift = new LinkagePID(hardwareMap);
        LinkagePID2 lift2 = new LinkagePID2(hardwareMap);
        ServoPID servoPID = new ServoPID();
        GlobalUse global = new GlobalUse();
        global.SetLiftTarget(0);
        robot.setBratTarget(0);
        runtime.reset();
        waitForStart();
        robot.limeLight.pipelineSwitch(0);


        while (opModeIsActive()) {
            robot.DriveMovement(gamepad1);
            robot.BratPID(gamepad2);
            robot.ubratTarget(gamepad2);
            robot.ClawManager(gamepad2);
            robot.BratServo(gamepad2);
            robot.ClawRotation(gamepad2);
            robot.ClawYManager(gamepad2);
            lift.LiftPIDControlStanga(gamepad2);
            lift2.LiftControlRight(gamepad2);
            //global.LiftManualControl(gamepad2 , lift.MotorLiftStanga , lift2.MotorLiftDreapta);
           // robot.Limelight(telemetry);
           // telemetry.addData("BratSvTarget" , servoPID.ReturnBratSvTarget());
           // telemetry.addData("BratServoLeftPos" , robot.ReturnAnalogBratLeft()); ///132-pickUp 260-basket
           // telemetry.addData("BratServoRightPos" , robot.ReturnAnalogBratRight());
            telemetry.addData("Arm Target", robot.ReturnPosBrat());
            telemetry.addData("Arm Pos", robot.Brat.getCurrentPosition());
            telemetry.addData("Lift Target" , lift.ReturnLiftTarget());
            telemetry.addData("Lift Left Pos" , lift.MotorLiftStanga.getCurrentPosition());
            telemetry.addData("Lift Right Pos" , lift2.MotorLiftDreapta.getCurrentPosition());
            telemetry.addData("Linkage Status" , global.ReturnLinkageStatus());
            telemetry.addData("Runtime Seconds - ", runtime.seconds());
            telemetry.update();

        }
    }
}

