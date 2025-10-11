package org.firstinspires.ftc.teamcode.DummyClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor intake;

    public Intake (HardwareMap hardwareMap){
        intake = hardwareMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void start(double speed){
        intake.setPower(speed);
    }

    public void stop(){
        intake.setPower(0);
    }


}
