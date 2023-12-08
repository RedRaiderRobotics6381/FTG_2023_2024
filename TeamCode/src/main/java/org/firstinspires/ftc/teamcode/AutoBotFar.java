package org.firstinspires.ftc.teamcode;

public class AutoBotFar extends FO_MechanumOpMode{
    Auto DriveTrain = new Auto();
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain = new Auto();
        waitForStart();
        if (isStopRequested()) return;
        DriveTrain.Drive("Forward", 4000);
        DriveTrain.Strafe("Right", 7000);
        DriveTrain.Drive("Reverse", 4000);
        DriveTrain.Strafe("Right", 3000);
    }
}
