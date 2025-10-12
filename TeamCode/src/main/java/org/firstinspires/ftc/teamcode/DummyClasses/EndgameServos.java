package org.firstinspires.ftc.teamcode.DummyClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class EndgameServos {
    Servo sL;
    Servo sR;
    public static double sLPos = 0.43;
    public static double sRPos = 0.43;

    public EndgameServos(HardwareMap hardwareMap){
        sL = hardwareMap.servo.get("liftServoLeft");
        sR =  hardwareMap.servo.get("liftServoRight");
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
}