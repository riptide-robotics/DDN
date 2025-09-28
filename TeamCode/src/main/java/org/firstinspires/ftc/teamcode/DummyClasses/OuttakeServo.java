package org.firstinspires.ftc.teamcode.DummyClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeServo {

    Servo outtakeServo;

    public OuttakeServo(HardwareMap hardwareMap){
        outtakeServo = hardwareMap.servo.get("outtakePitch");
    }
    
    public void controlPitch(double angle){
        outtakeServo.setPosition(angle);
    }
}