package org.firstinspires.ftc.teamcode.AutoPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutoSubsystems.Auto;
import org.firstinspires.ftc.teamcode.AutoSubsystems.Lift;

@Autonomous
public class AutoBotFarBlue extends LinearOpMode {
    Auto DriveTrain;
    Lift Will = new Lift(hardwareMap);
    Servo rightOuttake = hardwareMap.servo.get("rightOuttake");
    Servo leftOuttake = hardwareMap.servo.get("leftOuttake");
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain = new Auto(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        DriveTrain.Drive("Forward", 137.16);
        DriveTrain.Strafe("Left", 243.84);
        DriveTrain.Drive("Reverse", 60.96);
        DriveTrain.Turn("Left", 90);
        Will.middle();
        rightOuttake.setPosition(1);
        leftOuttake.setPosition(1);
        Will.down();
        DriveTrain.Strafe("Left", 60.96);
        DriveTrain.Drive("Forward", 60.96);
    }
}
