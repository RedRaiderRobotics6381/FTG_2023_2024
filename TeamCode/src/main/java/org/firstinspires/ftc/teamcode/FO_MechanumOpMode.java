
package org.firstinspires.ftc.teamcode;
import android.util.Log;
import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.apriltag.AprilTagDetection;
import java.text.DecimalFormat;
@TeleOp
public class FO_MechanumOpMode extends LinearOpMode {
    OpenCvWebcam webcam;
    boolean armUp = false;
    DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");
     public boolean keepGoing = false;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo Crabtake1 = hardwareMap.servo.get("Crabtake1");
        Servo Crabtake2 = hardwareMap.servo.get("Crabtake2");
        Servo Crabtake3 = hardwareMap.servo.get("Crabtake3");
        Servo Crabtake4 = hardwareMap.servo.get("Crabtake4");
        Lift Will = new Lift();
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
        double y = -gamepad1.left_stick_y * z; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * z;
        double rx = gamepad1.right_stick_x * z;
        boolean calibration_complete = false;
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Crabtake1.setDirection(Servo.Direction.REVERSE);
        Crabtake3.setDirection(Servo.Direction.REVERSE);
        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData, NAVX_DEVICE_UPDATE_RATE_HZ);
        yawPIDController = new navXPIDController( navx_device, navXPIDController.navXTimestampedDataSource.YAW);
        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        // Webcam Stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new SamplePipeline());
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode){
            }
        });
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            telemetry.addData("armMotor-encoder", armMotor.getCurrentPosition());
            telemetry.addData("liftMotor-encoder", liftMotor.getCurrentPosition());
            telemetry.update();
            y = -gamepad1.left_stick_y * z; // Remember, Y stick value is reversed
            x = gamepad1.left_stick_x * z;
            rx = gamepad1.right_stick_x * z;
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.start) {
                navx_device.zeroYaw();
            }
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
            double botHeading = navx_device.getYaw();
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            /*double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing
            Denominator is the largest motor power (absolute value) or 1
            This ensures all the powers maintain the same ratio,
            but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;*/
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            if (gamepad2.a){
                Crabtake1.setPosition(0.5);
                Crabtake2.setPosition(0.5);
            }
            if (gamepad2.dpad_up) {

                    if (armMotor.getCurrentPosition() >= 10000){
                        armMotor.setPower(-1);
                    }
                    else if (armMotor.getCurrentPosition() <= 0) {
                        armMotor.setPower(1);
                    }
                }

                    if (armMotor.getCurrentPosition() > 10000 && armMotor.getPower() > 0) {
                        armMotor.setPower(0);
                    }
                    if (armMotor.getCurrentPosition() < 0 && armMotor.getPower() < 0) {
                        armMotor.setPower(0);
                    }

                        if (gamepad2.right_stick_y >= 0.1 || gamepad2.right_stick_y <= -0.1) {
                            armMotor.setPower(-gamepad2.right_stick_y);
                        }
                        if (gamepad1.left_trigger >= 0.75) {
                            intakeMotor.setPower(1);
                        } else if (gamepad1.right_trigger >= 0.75) {
                            intakeMotor.setPower(-0.5);
                        } else {
                            intakeMotor.setPower(0);
                        }
                        if (gamepad1.left_bumper) {
                            z = 0.25;
                        } else if (gamepad1.right_bumper) {
                            z = 1;
                        } else {
                            z = 0.75;
                        }
                        if (gamepad1.a) {
                            webcam.stopStreaming();
                        }
                        /** test to find upper and lower limits
                         * if(liftMotor.getCurrentPosition <= (upper limit) && liftMotor.getCurrentPosition ></=>= (lower limit)){
                         *   liftMotor.setPower(gamepad2.left_stick_y);
                         * } else if (liftMotor.getCurrentPosition > (upper limit)){
                         *liftMotor.setPower(-Math.abs(gamepad2.left_stick_y));
                         * } else if (liftMotor.getCurrentPosition > (upper limit)){
                         * liftMotor.setPower(Math.abs(gamepad2.left_stick_y)););
                         * }
                         */
                        if (gamepad2.a){
                            Will.down();
                        } else if (gamepad2.b){
                            Will.middle();
                        } else if (gamepad2.x){
                            Will.top();
                        }

            if (gamepad2.left_stick_y >= 0.1 || gamepad2.left_stick_y <= -0.1) {
                liftMotor.setPower(-gamepad2.left_stick_y);
                sleep(10);
                liftMotor.setPower(0);
            }
            if (liftMotor.getCurrentPosition() > 2900 && liftMotor.getPower() > 0) {
                liftMotor.setPower(0);
            }
            if (liftMotor.getCurrentPosition() < 0 && liftMotor.getPower() < 0) {
                liftMotor.setPower(0);
            }
            if (!keepGoing) {
                if (liftMotor.getCurrentPosition() < 1600 && liftMotor.getCurrentPosition() > 1400) {
                    liftMotor.setPower(0);
                    keepGoing = true;
                }
            } else {

            }

                        }
                    }
                    class SamplePipeline extends OpenCvPipeline {
                        boolean viewportPaused;

                        @Override
                        public Mat processFrame(Mat input) {
                            Imgproc.rectangle(
                                    input,
                                    new Point(
                                            input.cols() / 4.0,
                                            input.rows() / 4.0),
                                    new Point(
                                            input.cols() * (3f / 4f),
                                            input.rows() * (3f / 4f)),
                                    new Scalar(0, 255, 0), 4);
                            return input;
                        }

                        @Override
                        public void onViewportTapped() {
                            viewportPaused = !viewportPaused;
                            if (viewportPaused) {
                                webcam.pauseViewport();
                            } else {
                                webcam.resumeViewport();
                            }
                        }
                    }
                }