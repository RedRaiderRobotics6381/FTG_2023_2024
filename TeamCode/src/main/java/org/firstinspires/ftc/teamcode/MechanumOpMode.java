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
import java.text.DecimalFormat;
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
    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
    DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");
    DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");

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

    boolean calibration_complete = false;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData, NAVX_DEVICE_UPDATE_RATE_HZ);
        yawPIDController = new navXPIDController( navx_device, navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);


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


            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {
                while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
                    calibration_complete = !navx_device.isCalibrating();
                    if (!calibration_complete) {
                        telemetry.addData("navX-Micro", "Startup Calibration in Progress");
                    }
                }
                navx_device.zeroYaw();

                try {
                    yawPIDController.enable(true);

        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

                    final double TOTAL_RUN_TIME_SECONDS = 30.0;
                    int DEVICE_TIMEOUT_MS = 500;
                    navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

                    DecimalFormat df = new DecimalFormat("#.##");

                    while ( (runtime.time() < TOTAL_RUN_TIME_SECONDS) &&
                            !Thread.currentThread().isInterrupted()) {
                        if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                            if (yawPIDResult.isOnTarget()) {
                                //leftMotor.setPowerFloat();
                                //rightMotor.setPowerFloat();
                                telemetry.addData("PIDOutput", df.format(0.00));
                            } else {
                                double output = yawPIDResult.getOutput();
                                //leftMotor.setPower(output);
                                //rightMotor.setPower(-output);
                                telemetry.addData("PIDOutput", df.format(output) + ", " +
                                        df.format(-output));
                            }
                        } else {
                            /* A timeout occurred */
                            Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                        }
                        telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                    }
                }
                catch(InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }
                finally {
                    navx_device.close();
                    telemetry.addData("LinearOp", "Complete");
                }



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



