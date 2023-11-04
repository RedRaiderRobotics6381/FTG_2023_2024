package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import java.lang.Object;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp
public class MechanumOpMode extends LinearOpMode {
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope) navxMicro;

        double z = 0.75;
        double y = gamepad1.left_stick_y * z;
        double x = -gamepad1.left_stick_x * z;
        double rx = -gamepad1.right_stick_x * z;
        boolean armUp = false;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);


            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {

                double rcw = pJoystick->GetTwist();
                double forwrd = pJoystick->GetY() * -1; /* Invert stick Y axis */
                double strafe = pJoystick->GetX();

                double pi = 3.1415926;

                /* Adjust Joystick X/Y inputs by navX MXP yaw angle */

                double gyro_degrees = ahrs->GetYaw();
                double gyro_radians = gyro_degrees * pi/180;
                double temp = forwrd * cos(gyro_radians) +
                        strafe * sin(gyro_radians);
                strafe = -forwrd * sin(gyro_radians) +
                        strafe * cos(gyro_radians);
                fwd = temp;

                /* At this point, Joystick X/Y (strafe/forwrd) vectors have been */
                /* rotated by the gyro angle, and can be sent to drive system */

                telemetry.log().add("Gyro Calibrating. Do Not Move!");

                // Wait until the gyro calibration is complete
                timer.reset();
                while (navxMicro.isCalibrating()) {
                    telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
                    telemetry.update();
                    Thread.sleep(50);
                }
                telemetry.log().clear();
                telemetry.log().add("Gyro Calibrated. Press Start.");
                telemetry.clear();
                telemetry.update();

                // Wait for the start button to be pressed
                waitForStart();
                telemetry.log().clear();

                // Read dimensionalized data from the gyro. This gyro can report angular velocities
                // about all three axes. Additionally, it internally integrates the Z axis to
                // be able to report an absolute angular Z orientation.
                AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
                Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                telemetry.addLine()
                        .addData("dx", formatRate(rates.xRotationRate))
                        .addData("dy", formatRate(rates.yRotationRate))
                        .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));

                telemetry.addLine()
                        .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
                        .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
                        .addData("pitch", "%s deg", formatAngle(angles.angleUnit, angles.thirdAngle));
                telemetry.update();

                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop


                telemetry.update();
                y = gamepad1.left_stick_y * z;
                x = -gamepad1.left_stick_x * z;
                rx = -gamepad1.right_stick_x * z;
                liftMotor.setPower(gamepad2.left_stick_y * 0.25);
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                frontLeftPower = (y + x + rx) / denominator;
                backLeftPower = (y - x + rx) / denominator;
                frontRightPower = (y - x - rx) / denominator;
                backRightPower = (y + x - rx) / denominator;

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

                telemetry.addData("armMotor-encoder", armMotor.getCurrentPosition());
                telemetry.update();

                if (gamepad1.right_bumper) {
                    z = 0.25;
                } else if (gamepad1.left_bumper) {
                    z = 1;
                } else {
                    z = 0.75;
                }
                /**Odemetry Notes:
                 *frontRightMotor Encoder == Odemetry Right
                 *backRightMotor Encoder == Odemetry Back
                 *frontLeftMotor Encoder == Odemetry Left
                 */

//            if (gamepad2.) {
//                armMotor.setPower(-0.25);
//                sleep(1000);
//                armMotor.setPower(0);
//            }

            }
        }
    }



