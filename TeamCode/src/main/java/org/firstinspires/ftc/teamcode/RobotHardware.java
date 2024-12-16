package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Collections;
import java.util.List;

@Config
public class RobotHardware {
    public DcMotorEx
            leftFront, leftRear,
            rightRear, rightFront,
            Brat, MotorTest;

    public Servo
            Gheara , ClawRotate , ClawY;

    public Servo BratServoLeft , BratServoRight;

    public AnalogInput BratAnalogLeft , BratAnalogRight;
    public static double BratDownPos = 0.46 , BratUpPose = 0.20;
    public static double analogPosLeft , analogPosRight;
    GlobalUse global = new GlobalUse();

    public Limelight3A limeLight;
    public static double openPos = 0.9;
    public static double closePos = 0.1;

    public double SampleAngle;
    public String SampleDirection;

    ///Lift PID
    public static int LiftTarget;

    ///Claw
    boolean ghearaOpen = true;
    boolean buttonIsPressed = false, toggleClaw = false;

    ///Brat
    boolean buttonIsPressedBrat = false , toogleBrat=false;


    boolean manualControlBrat = false;
    public double pid;
    public IMU imu;
    PIDController pidController = new PIDController(0, 0, 0);

    public static double kpBrat = 0.0035, kiBrat = 0, kdBrat = 0, ffBrat = 0.01;

    public static int BratTarget = 0;

    public RobotHardware(HardwareMap hardwareMap) {

        leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "RightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);


        Gheara = hardwareMap.get(Servo.class, "Gheara");
        ClawRotate = hardwareMap.get(Servo.class , "GhearaRotatie");

        ClawY = hardwareMap.get(Servo.class, "GhearaY");



        ///Brat
        Brat = hardwareMap.get(DcMotorEx.class, "Brat");
        Brat.setDirection(DcMotorEx.Direction.REVERSE);
        Brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Brat.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Brat.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        ///Limelight
        limeLight = hardwareMap.get(Limelight3A.class, "limelight");
        limeLight.setPollRateHz(250);


        ///MotorTest
        MotorTest = hardwareMap.get(DcMotorEx.class, "MotorTest");
        MotorTest.setDirection((DcMotorSimple.Direction.FORWARD));
        MotorTest.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorTest.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        ///BratServo
        BratServoLeft = hardwareMap.get(Servo.class , "BratServoLeft");
        BratServoRight = hardwareMap.get(Servo.class , "BratServoRight");
       // BratServoLeft.setDirection(DcMotorSimple.Direction.FORWARD);
       // BratServoRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BratAnalogLeft = hardwareMap.get(AnalogInput.class , "BratAnalogInputLeft");
        BratAnalogRight = hardwareMap.get(AnalogInput.class , "BratAnalogInputRight");

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }



    public void DriveMovement(Gamepad gamepad) {
//        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
//        double x = gamepad.left_stick_x;
//        double rx = gamepad.right_stick_x;
//
//        if (!gamepad.left_bumper) {
//            x /= 2;
//            y /= 2;
//        }
//        if (!gamepad.right_bumper) {
//            rx /= 2;
//        }
//
//        if (gamepad.options) {
//            imu.resetYaw();
//        }
//
//        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//        double frontLeftPower = (rotY + rotX + rx);
//        double backLeftPower = (rotY - rotX + rx);
//        double frontRightPower = (rotY - rotX - rx);
//        double backRightPower = (rotY + rotX - rx);
//
//        leftFront.setPower(frontLeftPower);
//        leftRear.setPower(backLeftPower);
//        rightFront.setPower(frontRightPower);
//        rightRear.setPower(backRightPower);

        double y = -gamepad.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        if (!gamepad.left_bumper) {
            x /= 2;
            y /= 2;
        }
        if (!gamepad.right_bumper) {
            rx /= 2;
        }

        leftFront.setPower(y + x + rx);
        leftRear.setPower(y - x + rx);
        rightFront.setPower(y - x - rx);
        rightRear.setPower(y + x - rx);
    }

    public double ReturnAnalogBratLeft()
    {
        return analogPosLeft;
    }
    public double ReturnAnalogBratRight()
    {
        return analogPosRight;
    }
    public void BratPID(Gamepad gamepad) {
        pidController.setPID(kpBrat, kiBrat, kdBrat);
        int BratPos = Brat.getCurrentPosition();
        double pid = pidController.calculate(BratPos, BratTarget);
        double pidPowerBRAT = pid + ffBrat;
        Brat.setPower(pidPowerBRAT);

    }

    public int ReturnPosBrat() {
        return BratTarget;
    }

    public void setBratTarget(int a) {
        BratTarget = a;
    }

    public void ubratTarget(Gamepad gamepad)
    {
        if(gamepad.dpad_up && gamepad.left_bumper)
        {
            setBratTarget(global.linkageUpPose);
            global.ChangeLinkageStatus(false);
        }
        if (gamepad.dpad_down && gamepad.left_bumper)
        {
            setBratTarget(global.linkageDownPos);
            global.ChangeLinkageStatus(true);
        }

    }

    boolean buttonY=false;
    boolean BratOpen=false;
    public void BratServo(Gamepad gamepad)
    {
//        analogPosLeft = BratAnalogLeft.getVoltage()/3.3*360;
//        analogPosRight = BratAnalogRight.getVoltage()/3.3*360;
//
//        if(gamepad.x && gamepad.right_bumper)
//        {
//            servoPID.SetBratSvTarget(132);
//            servoPID.PIDServo(BratServoLeft , BratAnalogLeft);
//            servoPID.PIDServo(BratServoRight , BratAnalogRight);
//        }
//        else
//            if(gamepad.b && gamepad.right_bumper)
//            {
//                servoPID.SetBratSvTarget(260);
//                servoPID.PIDServo(BratServoLeft , BratAnalogLeft);
//                servoPID.PIDServo(BratServoRight , BratAnalogRight);
//            }

        if (gamepad.y && !buttonY) {
            BratOpen = !BratOpen;
            if (BratOpen) {
                BratServoLeft.setPosition(BratDownPos);
                BratServoRight.setPosition(BratUpPose);
            } else {
                BratServoLeft.setPosition(BratUpPose);
                BratServoRight.setPosition(BratDownPos);
            }
            buttonY = true;
        }
        if (!gamepad.y) {
            buttonY = false;
        }
    }

    public void Limelight(Telemetry telemetry) {
        LLResult result = limeLight.getLatestResult();
        if (result.isValid())
        {
            double[] outputs = result.getPythonOutput();
            if(outputs!=null && outputs.length>0)
            {
                SampleAngle=outputs[1];

            }
        }

    }
public void ClawManager(Gamepad gamepad) {
    if (gamepad.a && !buttonIsPressed) {
        ghearaOpen = !ghearaOpen;
        if (ghearaOpen) {
            Gheara.setPosition(openPos);
        } else {
            Gheara.setPosition(closePos);
        }
        buttonIsPressed = true;
    }
    if (!gamepad.a) {
        buttonIsPressed = false;
    }
}

    boolean open=false , button=false;
    public void ClawRotation(Gamepad gamepad)
    {
        if (gamepad.b && !button && !gamepad.right_bumper) {
            open = !open;
            if (open) {
                ClawRotate.setPosition(1);
            } else {
                ClawRotate.setPosition(0.5);
            }
            button = true;
        }
        if (!gamepad.b && !gamepad.right_bumper) {
            button = false;
        }

    }

    boolean buttonX=false;
    boolean ClawOpen=false;

    public void ClawYManager(Gamepad gamepad)
    {
        if (gamepad.x && !buttonX && !gamepad.right_bumper) {
        ClawOpen = !ClawOpen;
        if (ClawOpen) {
            ClawY.setPosition(1);
        } else {
            ClawY.setPosition(0);
        }
        buttonX = true;
    }
        if (!gamepad.x && !gamepad.right_bumper) {
            buttonX = false;
        }
    }
}



