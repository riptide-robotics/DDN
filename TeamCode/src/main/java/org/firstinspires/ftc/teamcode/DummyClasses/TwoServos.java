package org.firstinspires.ftc.teamcode.DummyClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.Serializable;

public class TwoServos {
    Servo s1;
    Servo s2;
    public static double s1Pos = 0.43;
    public static double s2Pos = 0.43;

    public TwoServos(HardwareMap hardwareMap){
        s1 = hardwareMap.servo.get("s1");
        s2 =  hardwareMap.servo.get("s2");
        s2.setDirection(Servo.Direction.REVERSE);
    }

    public void lift(){
        s1.setPosition(s1Pos);
        s2.setPosition(s2Pos);
    }
    public void lower(){
        s1.setPosition(0);
        s2.setPosition(0);
    }
}
