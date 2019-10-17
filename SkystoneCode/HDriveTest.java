package org.firstinspires.ftc.teamcode.SkystoneCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="HDriveTest")
public class HDriveTest extends OpMode {

    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor middleWheelMotor;

    @Override
    public void init() {
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        middleWheelMotor = hardwareMap.get(DcMotor.class, "middleWheelMotor");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.right_stick_y > 0.01 || gamepad1.right_stick_y < -0.01) {

            frontRightMotor.setPower(gamepad1.right_stick_y);
            backRightMotor.setPower(gamepad1.right_stick_y );

        } else {
            frontRightMotor.setPower(0.00);
            backRightMotor.setPower(0.00);
        }

        if (gamepad1.left_stick_y > 0.01 || gamepad1.left_stick_y < -0.01) {

            frontLeftMotor.setPower(gamepad1.left_stick_y);
            backLeftMotor.setPower(gamepad1.left_stick_y);
        } else {
            frontLeftMotor.setPower(0.00);
            backLeftMotor.setPower(0.00);
        }

        if (gamepad1.left_bumper){
            middleWheelMotor.setPower(-1.0);
        } else if (gamepad1.right_bumper){
            middleWheelMotor.setPower(1.0);
        } else {
            middleWheelMotor.setPower(0.0);
        }
    }
}
