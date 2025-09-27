package org.firstinspires.ftc.teamcode.DummyClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

public class Outtake {
    private DcMotor motorL;
    private DcMotor motorR;

    public Outtake(HardwareMap hardwareMap){

        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");

        motorL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void start(int speed){
        motorR.setPower(speed);
        motorL.setPower(speed);
    }

    public void stop(){
        motorR.setPower(0);
        motorL.setPower(0);
    }

}
