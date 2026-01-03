package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
// make some kind of program so we can test which angles work
public class EndgameServos {
    Servo sL;
    Servo sR;
    public static double sLPos = 0.43;
    public static double sRPos = 0.43;

    public EndgameServos(HardwareMap hardwareMap){
        sL = hardwareMap.servo.get("liftServoLeft");
        sR = hardwareMap.servo.get("liftServoRight");

        sL.setDirection(Servo.Direction.FORWARD);
        sR.setDirection(Servo.Direction.REVERSE);

    }

    public void lift(){
        sL.setPosition(sLPos);
        sR.setPosition(sRPos);
    }
    public void lower(){
        sL.setPosition(0);
        sR.setPosition(0);
    }

    public void testLift(double lift) {
        sL.setPosition(lift);
        sR.setPosition(lift);
    }
    public void testLower(double zero) {
        sL.setPosition(zero);
        sR.setPosition(zero);
    }
}