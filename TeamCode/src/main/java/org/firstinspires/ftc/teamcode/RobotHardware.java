package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class RobotHardware {
    public DcMotorEx
            leftFront, leftRear,
            rightRear, rightFront,
            Brat, MotorLiftStanga , MotorLiftDreapta , MotorTest;

    public Servo
            Gheara , ClawRotate , clawY;

    public Servo BratServoLeft , BratServoRight;

    GlobalUse global = new GlobalUse();
    ClawY ClawY = new ClawY();
    ClawX clawX = new ClawX();
    Claw claw = new Claw();
    BratServo bratServo = new BratServo();
    Linkage linkage = new Linkage();

    DriveTrain driveTrain = new DriveTrain();

    public Limelight3A limeLight;

    public IMU imu;
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

        clawY = hardwareMap.get(Servo.class, "GhearaY");

        ///Lift
        MotorLiftDreapta = hardwareMap.get(DcMotorEx.class, "LiftDreapta");
        MotorLiftDreapta.setDirection(DcMotorEx.Direction.FORWARD);
        MotorLiftDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorLiftDreapta.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        MotorLiftStanga = hardwareMap.get(DcMotorEx.class, "LiftStanga");
        MotorLiftStanga.setDirection(DcMotorEx.Direction.REVERSE);
        MotorLiftStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorLiftStanga.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MotorLiftStanga.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MotorLiftStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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


        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }



    public void LiftControl(Gamepad gamepad)
    {
        global.LiftControl(gamepad , MotorLiftStanga , MotorLiftDreapta , ReturnLinkageStatus());
    }
    public void DriveMovement(Gamepad gamepad)
    {
        driveTrain.DriveMovement(gamepad , leftFront , leftRear , rightFront , rightRear);
    }
    public void LinkagePID(Gamepad gamepad)
    {
        linkage.LinkagePID(gamepad , Brat);
    }

    public int ReturnLinkageTarget(){return linkage.ReturnLinkageTarget();}
    public boolean ReturnLinkageStatus(){return linkage.ReturnLinkageStatus();}
    public void BratServo(Gamepad gamepad) {bratServo.BratManager(gamepad , BratServoLeft , BratServoRight);}

//    public void Limelight(Telemetry telemetry) {
//        LLResult result = limeLight.getLatestResult();
//        if (result.isValid())
//        {
//            double[] outputs = result.getPythonOutput();
//            if(outputs!=null && outputs.length>0)
//            {
//                SampleAngle=outputs[1];
//
//            }
//        }
//
//    }

    public void ClawManager(Gamepad gamepad)
{
    claw.ClawManager(gamepad , Gheara);
}
    public void ClawRotation(Gamepad gamepad)
    {
        clawX.ClawXManager(gamepad , ClawRotate);
    }
    public void ClawYManager(Gamepad gamepad)
    {
        ClawY.ClawYManager(gamepad , clawY);
    }
}



