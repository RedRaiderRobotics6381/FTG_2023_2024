package org.firstinspires.ftc.teamcode;

public class Auto extends FO_MechanumOpMode{
    double x = 0.25;
    double RPM = 312 * x;
    double pi = 3.14159;
    double cmPerSec = (RPM / 60) * 9.6 * pi;
    double secPerCm = 1 / cmPerSec;
    double degreesPerSecond = (RPM / 60) * 528 * 360;
    double secondsPerDegree = 1 / degreesPerSecond;

    public void Drive(String direction, double centimeters){
        long milliseconds = Math.round(centimeters * secPerCm) * 1000;
        if (direction.equals("Forward")){
            frontLeftMotor.setPower(x);
            frontRightMotor.setPower(x);
            backLeftMotor.setPower(x);
            backRightMotor.setPower(x);
            sleep(milliseconds);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

        } else if (direction.equals("Reverse")){
            frontLeftMotor.setPower(-x);
            frontRightMotor.setPower(-x);
            backLeftMotor.setPower(-x);
            backRightMotor.setPower(-x);
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
            frontLeftMotor.setPower(-x);
            frontRightMotor.setPower(x);
            backLeftMotor.setPower(x);
            backRightMotor.setPower(-x);
            sleep(milliseconds);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

        } else if (direction.equals("Right")){
            frontLeftMotor.setPower(x);
            frontRightMotor.setPower(-x);
            backLeftMotor.setPower(-x);
            backRightMotor.setPower(x);
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
            frontLeftMotor.setPower(-x);
            frontRightMotor.setPower(x);
            backLeftMotor.setPower(-x);
            backRightMotor.setPower(x);
            sleep(milliseconds);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

        } else if (direction.equals("Right")){
            frontLeftMotor.setPower(x);
            frontRightMotor.setPower(-x);
            backLeftMotor.setPower(x);
            backRightMotor.setPower(-x);
            sleep(milliseconds);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }
    public void setSpeed(double percent){
        x = percent;
    }
}