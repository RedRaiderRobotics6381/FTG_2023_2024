package org.firstinspires.ftc.teamcode;


import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class Auto{
    public double SpeedOfAuto = 0.25;
    double RPM = 312 * SpeedOfAuto;

    double cmPerSec = (RPM / 60) * 9.6 * Math.PI;
    double secPerCm = 1 / cmPerSec;
    double degreesPerSecond = (RPM / 60) * 528 * 360;
    double secondsPerDegree = 1 / degreesPerSecond;
    private DcMotor frontRightMotor, backRightMotor, backLeftMotor, frontLeftMotor;

    public Auto(HardwareMap hardwareMap){
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void Drive(String direction, double centimeters) {
        long milliseconds = Math.round(centimeters * secPerCm) * 1000;

            if (direction.equals("Forward")) {
                frontLeftMotor.setPower(SpeedOfAuto);
                backLeftMotor.setPower(SpeedOfAuto);
                frontRightMotor.setPower(SpeedOfAuto);
                backRightMotor.setPower(SpeedOfAuto);


                sleep(milliseconds);

                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);

            } else if (direction.equals("Reverse")) {
                frontLeftMotor.setPower(-SpeedOfAuto);
                frontRightMotor.setPower(-SpeedOfAuto);
                backLeftMotor.setPower(-SpeedOfAuto);
                backRightMotor.setPower(-SpeedOfAuto);
                sleep(milliseconds);

                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
            }
        }

    public void Strafe(String direction, double centimeters){

        long milliseconds = Math.round(centimeters * secPerCm * (10.0/ 9.0)) * 1000;

        if (direction.equals("Left")){
            frontLeftMotor.setPower(-SpeedOfAuto);
            frontRightMotor.setPower(SpeedOfAuto);
            backLeftMotor.setPower(SpeedOfAuto);
            backRightMotor.setPower(-SpeedOfAuto);
            sleep(milliseconds);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

        } else if (direction.equals("Right")) {
            frontLeftMotor.setPower(SpeedOfAuto);
            frontRightMotor.setPower(-SpeedOfAuto);
            backLeftMotor.setPower(-SpeedOfAuto);
            backRightMotor.setPower(SpeedOfAuto);
            sleep(milliseconds);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }

    public void Turn(String direction, double degrees){
        long milliseconds = Math.round(degrees * secondsPerDegree);

        if (direction.equals("Left")){
            frontLeftMotor.setPower(-SpeedOfAuto);
            frontRightMotor.setPower(SpeedOfAuto);
            backLeftMotor.setPower(-SpeedOfAuto);
            backRightMotor.setPower(SpeedOfAuto);
            sleep(milliseconds);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

        } else if (direction.equals("Right")) {
            frontLeftMotor.setPower(SpeedOfAuto);
            frontRightMotor.setPower(-SpeedOfAuto);
            backLeftMotor.setPower(SpeedOfAuto);
            backRightMotor.setPower(-SpeedOfAuto);

            sleep(milliseconds);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
        }

    public void setSpeed(double percent) {
        SpeedOfAuto = percent;
    }
}