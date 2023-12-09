package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AutoSubsystems.Auto;

@Autonomous
public class AutoFoward extends LinearOpMode {
    private Auto DriveTrain = null;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain = new Auto(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            DriveTrain.Drive("Forward", 130);
        }
    }
}