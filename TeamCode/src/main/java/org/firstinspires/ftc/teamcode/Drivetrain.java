package org.firstinspires.ftc.teamcode;

public class Drivetrain extends FO_MechanumOpMode{
    public void DriveForward(String direction, int centimeters){
        if (direction.equals("forward")){
            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(1);
            backLeftMotor.setPower(1);
            backRightMotor.setPower(1);
        } else if (direction.equals("reverse")){
            frontLeftMotor.setPower(-1);
            frontRightMotor.setPower(-1);
            backLeftMotor.setPower(-1);
            backRightMotor.setPower(-1);
        }
    }
}
