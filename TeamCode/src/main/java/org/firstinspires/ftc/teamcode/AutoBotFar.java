package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.AutoSubsystems.Lift;
import org.firstinspires.ftc.teamcode.AutoSubsystems.Auto;

@Autonomous
public class AutoBotFar extends LinearOpMode {
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
        DriveTrain.Strafe("Right", 243.84);
        DriveTrain.Drive("Reverse", 60.96);
        DriveTrain.Turn("Right", 90);
        Will.middle();
        rightOuttake.setPosition(1);
        leftOuttake.setPosition(1);
        Will.down();
        DriveTrain.Strafe("Right", 60.96);
        DriveTrain.Drive("Forward", 60.96);
    }
}
