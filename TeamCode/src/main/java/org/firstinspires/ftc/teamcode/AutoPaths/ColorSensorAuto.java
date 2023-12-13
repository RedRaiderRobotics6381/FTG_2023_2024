package org.firstinspires.ftc.teamcode.AutoPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.AutoSubsystems.Lift;
import org.firstinspires.ftc.teamcode.AutoSubsystems.Auto;

@Autonomous
public class ColorSensorAuto extends LinearOpMode{
    Auto Drivetrain = null;
    ColorSensor color_sensor;
    public Servo rightClaw, leftClaw;
    int SkullPos = 0;
    int Check = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain = new Auto(hardwareMap);
        color_sensor = hardwareMap.colorSensor.get("color_sensor");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        leftClaw.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            Drivetrain.Drive("Forward", 76.2);
            if (color_sensor.red() >= 3000) {
                SkullPos = 2;
                Drivetrain.Drive("Forward", 30);
                leftClaw.setPosition(0);
            }
            if (SkullPos == 0) {
                Drivetrain.Turn("Left", 90);
                if (color_sensor.red() >= 3000) {
                    SkullPos = 1;
                    Drivetrain.Drive("Forward", 30);
                    leftClaw.setPosition(0);
                }
                if (SkullPos == 0) {
                    Drivetrain.Turn("Right", 180);
                    if (color_sensor.red() >= 3000) {
                        SkullPos = 3;
                        Drivetrain.Drive("Forward", 30);
                        leftClaw.setPosition(0);
                    }
                }
            }

            Drivetrain.Drive("Forward", 76.2);
            if (color_sensor.red() >= 3000){
                Check = 3;
                SkullPos = 2;
            }
            while (color_sensor.red() <= 3000 && Check <= 2) {
                Check++;
                Drivetrain.Turn("Left", 90);

                if (color_sensor.red() >= 3000){

                }
            }
        }
    }
}
