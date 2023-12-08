package org.firstinspires.ftc.teamcode;

public class Auto extends FO_MechanumOpMode{
    public void Drive(String direction, int seconds){
        seconds *= 1000;
        if (direction.equals("Forward")){
            frontLeftMotor.setPower(0.25);
            frontRightMotor.setPower(0.25);
            backLeftMotor.setPower(0.25);
            backRightMotor.setPower(0.25);
            sleep(seconds);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

        } else if (direction.equals("Reverse")){
            frontLeftMotor.setPower(-0.25);
            frontRightMotor.setPower(-0.25);
            backLeftMotor.setPower(-0.25);
            backRightMotor.setPower(-0.25);
            sleep(seconds);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }
    public void Strafe(String direction, int seconds){
        seconds *= 1000;
        if (direction.equals("Left")){
            frontLeftMotor.setPower(-0.25);
            frontRightMotor.setPower(0.25);
            backLeftMotor.setPower(0.25);
            backRightMotor.setPower(-0.25);
            sleep(seconds);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

        } else if (direction.equals("Right")){
            frontLeftMotor.setPower(0.25);
            frontRightMotor.setPower(-0.25);
            backLeftMotor.setPower(-0.25);
            backRightMotor.setPower(0.25);
            sleep(seconds);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }
    public void Turn(String direction, int seconds){
        seconds *= 1000;
        if (direction.equals("Left")){
            frontLeftMotor.setPower(-0.25);
            frontRightMotor.setPower(0.25);
            backLeftMotor.setPower(-0.25);
            backRightMotor.setPower(0.25);
            sleep(seconds);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

        } else if (direction.equals("Right")){
            frontLeftMotor.setPower(0.25);
            frontRightMotor.setPower(-0.25);
            backLeftMotor.setPower(0.25);
            backRightMotor.setPower(-0.25);
            sleep(seconds);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }
}