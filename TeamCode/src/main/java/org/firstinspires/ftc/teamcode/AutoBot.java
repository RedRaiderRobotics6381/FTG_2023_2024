package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoBot extends FO_MechanumOpMode{
    Auto DriveTrain = new Auto();
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain = new Auto();
        waitForStart();

        if (isStopRequested()) return;
        DriveTrain.Drive("Forward", 5);
}}