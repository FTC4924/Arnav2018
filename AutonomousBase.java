package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public abstract class AutonomousBase extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    Servo airhornServo;

    GyroSensor gyroSensor;

    List<Command> commands = new ArrayList<Command>();
    private Command currentCommand;
    private int commandIndex = 0;
    private double encoderAverage = 0;
    private double currentAngle = 0;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        airhornServo = hardwareMap.get(Servo.class, "servo");
        gyroSensor = hardwareMap.get(GyroSensor.class, "gyro");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        gyroSensor.calibrate();

        encoderAverage = (frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition() +
                backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition()) / 4;

        currentCommand = commands.get(commandIndex);
        startCommand(currentCommand);
    }

    @Override
    public void loop() {

        encoderAverage = (frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition() +
                backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition()) / 4;
        currentAngle = gyroSensor.getHeading();
        telemetry.addData("gyro: ", currentAngle);

        if (commandComplete(currentCommand) || commandIndex >= commands.size()) {
            commandIndex++;
            if (commandIndex >= commands.size()) {
                powerRightMotors(0);
                powerLeftMotors(0);
            } else {
                currentCommand = commands.get(commandIndex);
                startCommand(currentCommand);
            }
        } else {
            executeCommand(currentCommand);
        }
    }

    private void startCommand(Command command) {
        if (command instanceof Move) {
            command.startCommand(encoderAverage);
        }

        if (command instanceof Turn) {

        }
    }

    private boolean commandComplete(Command command) {
        return command.isComplete(encoderAverage);
    }

    private void executeCommand(Command command) {
        if (command instanceof Move) {
            powerRightMotors(((Move) command).getPower());
            powerLeftMotors(((Move) command).getPower());
        }

        if (command instanceof Turn) {
            powerRightMotors(((Turn) command).getPower());
            powerLeftMotors(-((Turn) command).getPower());
        }
    }

    private void powerRightMotors(double power) {
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    private void powerLeftMotors(double power) {
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
    }
}
