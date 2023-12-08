package org.firstinspires.ftc.teamcode;

public class AutoBotFar extends FO_MechanumOpMode{
    Auto DriveTrain = new Auto();

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain = new Auto();

        rightOuttake.setPosition(0.85);
        leftOuttake.setPosition(0.85);

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
