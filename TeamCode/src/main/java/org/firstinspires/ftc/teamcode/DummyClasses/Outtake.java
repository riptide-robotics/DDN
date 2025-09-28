package org.firstinspires.ftc.teamcode.DummyClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

public class Outtake {
    private final DcMotor motorL;
    private final DcMotor motorR;

    public Outtake(HardwareMap hardwareMap){

        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");

        motorL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void start(double speed){
        motorR.setPower(speed);
        motorL.setPower(speed);
    }

    public void stop(){
        motorR.setPower(0);
        motorL.setPower(0);
    }

    public double currPos(){
        double currPos = (double) (motorR.getCurrentPosition() + motorL.getCurrentPosition()) / 2;
        return currPos;
    }
}
