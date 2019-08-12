package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@TeleOp(name = "TankDriveSimple")
public class TankDriveSimple extends OpMode {

    //Declaring Motors, which are initialized in init() at program start
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    @Override
    public void init() {

        //Communicates with Robot Controller phone to assign electrical outputs to Motor
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        //These motors face the opposite direction. They are reversed so the robot drives straight
        frontRightMotor.setDirection(Direction.REVERSE);
        backRightMotor.setDirection(Direction.REVERSE);
    }

    @Override
    public void loop() {

        //Robot is controlled with "Tank Drive" using left and right joysticks
        moveLeftMotors(gamepad1.left_stick_y);
        moveRightMotors(gamepad1.right_stick_y);
    }

    //'speed' is between -1 and 1, inclusive
    public void moveLeftMotors(float speed) {

        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
    }

    //'speed' is between -1 and 1, inclusive
    public void moveRightMotors(float speed) {

        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }
}
