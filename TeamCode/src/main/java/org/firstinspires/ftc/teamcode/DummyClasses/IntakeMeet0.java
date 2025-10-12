package org.firstinspires.ftc.teamcode.DummyClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeMeet0 {

    DcMotor intake;

    public IntakeMeet0(HardwareMap hardwareMap) {
        intake = hardwareMap.dcMotor.get("intakeMotor");
        intake.setDirection(DcMotor.Direction.REVERSE);
    }

    public void start (){
        intake.setPower(1);
    }

    public void stop(){
        intake.setPower(0);
    }

}
