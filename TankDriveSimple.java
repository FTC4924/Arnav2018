package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;


@TeleOp(name = "TankDriveSimple")
public class TankDriveSimple extends OpMode {

    //Declaring Motors, which are initialized in init() at program start
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    Servo airhornServo;

    @Override
    public void init() {

        //Communicates with Robot Controller phone to assign electrical outputs to Motor
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        airhornServo = hardwareMap.get(Servo.class, "servo");

        //These motors face the opposite direction. They are reversed so the robot drives straight
        frontLeftMotor.setDirection(Direction.REVERSE);
        backLeftMotor.setDirection(Direction.REVERSE);
    }

    @Override
    public void loop() {
        telemetry.addData("stick", gamepad1.left_stick_y);
        telemetry.addData("trigger", gamepad1.right_trigger);

        //Robot is controlled with "Tank Drive" using left and right joysticks
        if (gamepad1.left_stick_y > 0.01 || gamepad1.left_stick_y < -0.01) {

            frontLeftMotor.setPower(gamepad1.left_stick_y);
            backLeftMotor.setPower(gamepad1.left_stick_y);
            frontRightMotor.setPower(gamepad1.left_stick_y);
            backRightMotor.setPower(gamepad1.left_stick_y);

        } else if (gamepad1.right_trigger > 0.0) {

            turn(-gamepad1.right_trigger);

        } else if (gamepad1.left_trigger > 0.0) {

            turn(gamepad1.left_trigger);

        } else {
            move(0.0f);
        }

        //Move airhornServo to 0.1 if button A is pressed and button B is not pressed
        if(gamepad1.a && !gamepad1.b) {

            airhornServo.setPosition(0.1);
        }

        //Move airhornServo to 0.9 if button B is pressed and button A is not pressed
        if(gamepad1.b && !gamepad1.a) {

            airhornServo.setPosition(0.4);
        }
    }

    //'speed' is between -1 and 1, inclusive
    public void move(float speed) {

        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    public void turn(float speed) {

        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(-speed);
        backLeftMotor.setPower(-speed);
    }
}