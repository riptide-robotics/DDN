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

        sL.setDirection(Servo.Direction.REVERSE);
        sR.setDirection(Servo.Direction.FORWARD);

    }

    public void lift(double pos){
        sL.setPosition(pos);
        sR.setPosition(pos);
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
    public double getPos() {
        return (sL.getPosition() + sR.getPosition()) / 2;
    }
}