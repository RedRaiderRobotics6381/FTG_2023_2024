package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;


@TeleOp
public class FO_MechanumOpMode extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        AHRS navx_device;
        navXPIDController yawPIDController;
        ElapsedTime runtime = new ElapsedTime();

        final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

        final double TARGET_ANGLE_DEGREES = 90.0;
        final double TOLERANCE_DEGREES = 2.0;
        final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
        final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
        final double YAW_PID_P = 0.005;
        final double YAW_PID_I = 0.0;
        final double YAW_PID_D = 0.0;
        double z = 0.75;
        double y = gamepad1.left_stick_y * z; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x * z;
        double rx = -gamepad1.right_stick_x * z;

        boolean armUp = false;

        boolean calibration_complete = false;
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData, NAVX_DEVICE_UPDATE_RATE_HZ);
        yawPIDController = new navXPIDController( navx_device, navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);

        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);

        // Retrieve the IMU from the hardware map




        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("armMotor-encoder", armMotor.getCurrentPosition());
            telemetry.update();
            y = -gamepad1.left_stick_y * z; // Remember, Y stick value is reversed
            x = gamepad1.left_stick_x * z;
            rx = gamepad1.right_stick_x * z;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                navx_device.zeroYaw();
            }
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            double botHeading = navx_device.getYaw();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);


            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.dpad_up) {
                if (!armUp) {
                    armMotor.setTargetPosition(11250);
                    armUp = true;
                } else {
                    armMotor.setTargetPosition(0);
                    armUp = false;
                }
            }

            if (gamepad1.right_bumper) {
                z = 0.25;
            } else if (gamepad1.left_bumper) {
                z = 1;
            } else {
                z = 0.75;
            }
        }
    }
}



