package org.firstinspires.ftc.teamcode.AutoSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
import org.firstinspires.ftc.teamcode.FO_MechanumOpMode;

public class Lift extends FO_MechanumOpMode {
    public DcMotor lift2Motor, liftMotor;
    public Lift(HardwareMap hardwareMap){
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        lift2Motor = hardwareMap.dcMotor.get("lift2Motor");
        lift2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
        public void down() {
            liftMotor.setTargetPosition(0);
            lift2Motor.setTargetPosition(0);
            liftMotor.setPower(0.1);
            lift2Motor.setPower(0.1);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
/*
            liftMotor.setTargetPosition(0);
          lift2Motor.setTargetPosition(0);
          while (liftMotor.getCurrentPosition() >= 0){
              liftMotor.setPower(-0.1);
              lift2Motor.setPower(-0.1);
          }
          liftMotor.setPower(0);
          lift2Motor.setPower(0);*/
     }
        public void middle() {
            liftMotor.setTargetPosition(1500);
            lift2Motor.setTargetPosition(1500);
            liftMotor.setPower(0.1);
            lift2Motor.setPower(0.1);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /*
            liftMotor.setTargetPosition(1500);
            lift2Motor.setTargetPosition(1500);
            while (liftMotor.getCurrentPosition() < 1400) {
                liftMotor.setPower(0.1);
                lift2Motor.setPower(0.1);
            }
            while (liftMotor.getCurrentPosition() > 1600) {
                liftMotor.setPower(-0.1);
                lift2Motor.setPower(-0.1);
            }
            liftMotor.setPower(0);
            lift2Motor.setPower(0);*/
        }
        public void top(){
            liftMotor.setTargetPosition(2900);
            lift2Motor.setTargetPosition(2900);
            liftMotor.setPower(0.1);
            lift2Motor.setPower(0.1);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            /*
            liftMotor.setTargetPosition(2900);
            lift2Motor.setTargetPosition(2900);
            while (liftMotor.getCurrentPosition() <= 2900){
                liftMotor.setPower(0.1);
                lift2Motor.setPower(0.1);
            }
            liftMotor.setPower(0);
            lift2Motor.setPower(0);*/
        }
    }