package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.FO_MechanumOpMode;

public class Lift extends FO_MechanumOpMode {


        public void down() {
            if (liftMotor.getCurrentPosition() < 0) {
                liftMotor.setPower(1);
                keepGoing = true;
            } else if (liftMotor.getCurrentPosition() > 0) {
                liftMotor.setPower(-1);
                keepGoing = true;
            }
        }
        public void middle() {
            if (liftMotor.getCurrentPosition() > 1500) {
                liftMotor.setPower(-1);
                keepGoing = false;
            } else if (liftMotor.getCurrentPosition() < 1500) {
                liftMotor.setPower(1);
                keepGoing = false;
            }
        }
        public void top(){
           if (liftMotor.getCurrentPosition() > 2900) {
               liftMotor.setPower(-1);
               keepGoing = true;
           } else if (liftMotor.getCurrentPosition() < 2900) {
               liftMotor.setPower(1);
               keepGoing = true;
           }
        }
    }
