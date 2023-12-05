package org.firstinspires.ftc.teamcode;

public class Lift2 extends FO_MechanumOpMode {


        public void down() {
            if (lift2Motor.getCurrentPosition() < 0) {
                lift2Motor.setPower(1);
                keepGoing = true;
            } else if (lift2Motor.getCurrentPosition() > 0) {
                lift2Motor.setPower(-1);
                keepGoing = true;
            }
        }
        public void middle() {
            if (lift2Motor.getCurrentPosition() > 1500) {
                lift2Motor.setPower(-1);
                keepGoing = false;
            } else if (lift2Motor.getCurrentPosition() < 1500) {
                lift2Motor.setPower(1);
                keepGoing = false;
            }
        }
        public void top(){
           if (lift2Motor.getCurrentPosition() > 2900) {
               lift2Motor.setPower(-1);
               keepGoing = true;
           } else if (lift2Motor.getCurrentPosition() < 2900) {
               lift2Motor.setPower(1);
               keepGoing = true;
           }
        }
    }
