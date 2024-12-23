package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.SystemClock;

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
        GlobalUse global = new GlobalUse();
        global.SetLiftTarget(0);
        robot.SetLinkageTarget(0);
        robot.SetLinkageStatus();
        robot.Init();
        runtime.reset();
        waitForStart();
        robot.limeLight.pipelineSwitch(0);


        while (opModeIsActive()) {
            robot.DriveMovement(gamepad1);
            robot.LinkagePID(gamepad2);
            robot.ClawManager(gamepad2);
            robot.BratServo(gamepad2);
            robot.ClawRotation(gamepad2);
            robot.ClawYManager(gamepad2);
            robot.LiftControl(gamepad2);
            telemetry.addData("Linkage Target", robot.ReturnLinkageTarget());
            telemetry.addData("Linkage Pos", robot.Brat.getCurrentPosition());
            telemetry.addData("Lift Target" , global.ReturnLiftTarget());
            telemetry.addData("Lift Pos" , robot.MotorLiftStanga.getCurrentPosition());
            telemetry.addData("Linkage Status" , robot.ReturnLinkageStatus());
            telemetry.addData("Runtime Seconds - ", runtime.seconds());
            telemetry.update();

        }
    }
}

