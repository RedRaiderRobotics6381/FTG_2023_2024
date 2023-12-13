package org.firstinspires.ftc.teamcode.AutoPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutoSubsystems.Auto;

@Autonomous
public class dropPixel extends LinearOpMode {
    private Auto DriveTrain = null;
    Servo rightClaw;
    Servo leftClaw;
    public Servo rightClawRot, leftClawRot;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain = new Auto(hardwareMap);
        rightClaw = hardwareMap.servo.get("rightClaw");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClawRot = hardwareMap.servo.get("rightClawRot");
        leftClawRot = hardwareMap.servo.get("leftClawRot");
        leftClaw.setPosition(-0.65);
        rightClaw.setPosition(0.65);
        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            rightClawRot.setPosition(0.47);
            leftClawRot.setPosition(0.47);
            DriveTrain.Drive("Forward", 103);
            leftClaw.setPosition(0);
            DriveTrain.Drive("Reverse", 95);

        }
    }
}
