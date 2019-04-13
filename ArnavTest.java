package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


@Autonomous(name = "ArnavTest Auto", group = "4924")

public class ArnavTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor extension;
    DcMotor rotation;

    boolean rotationOut = false;

    static BNO055IMU imu;

    @Override
    public void runOpMode() {
        extension = hardwareMap.get(DcMotor.class, "extension");
        rotation = hardwareMap.get(DcMotor.class, "rotation");

        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status:", "Starting");
        telemetry.update();

        while (opModeIsActive()) {

            rotation.setTargetPosition(3122);
            rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotation.setPower(1);
            if(!rotationOut) {
                if (rotation.getCurrentPosition() > 150) {
                    rotation.setPower(0);
                    extension.setPower(-1);
                    sleep(700);
                    extension.setPower(0);
                    rotationOut=true;

                }
            }

            if (rotationOut){
                rotation.setPower(1);
                if (rotation.getCurrentPosition() > 3100) {
                    rotation.setPower(0);
                }
            }

        }
    }
}