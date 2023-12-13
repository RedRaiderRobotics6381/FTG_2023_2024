package org.firstinspires.ftc.teamcode.AutoPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutoSubsystems.Auto;

@Autonomous
public class dropPixelX2 extends LinearOpMode {
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
        leftClaw.setPosition(0.61);
        rightClaw.setPosition(0.61);
        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            rightClawRot.setPosition(0.47);
            leftClawRot.setPosition(0.47);
            DriveTrain.Drive("Forward", 105.44);
            leftClaw.setPosition(0);
            DriveTrain.Turn("Left", 90);
            DriveTrain.Drive("Forward", 20);
            rightClaw.setPosition(0);
        }
    }
}
