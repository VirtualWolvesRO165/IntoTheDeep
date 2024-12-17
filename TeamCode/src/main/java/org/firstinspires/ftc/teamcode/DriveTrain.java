package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveTrain {

    public void DriveMovement(Gamepad gamepad , DcMotorEx leftFront , DcMotorEx leftRear , DcMotorEx rightFront , DcMotorEx rightRear) {
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

}
