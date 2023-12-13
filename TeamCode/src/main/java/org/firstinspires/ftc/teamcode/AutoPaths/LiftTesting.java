package org.firstinspires.ftc.teamcode.AutoPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AutoSubsystems.Auto;
import org.firstinspires.ftc.teamcode.AutoSubsystems.Lift;

@Autonomous
public class LiftTesting extends LinearOpMode {
    private Auto DriveTrain = null;
    private Lift Will = null;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain = new Auto(hardwareMap);
        Will = new Lift(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            Will.middle();
        }
    }
}