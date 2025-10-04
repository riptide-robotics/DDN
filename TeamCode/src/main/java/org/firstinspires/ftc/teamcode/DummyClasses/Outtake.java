package org.firstinspires.ftc.teamcode.DummyClasses;

import static org.firstinspires.ftc.teamcode.riptideUtil.FLYWHEEL_KP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Robot;

public class Outtake {
    private final DcMotor motorL;
    private final DcMotor motorR;
    private final Servo outtakeServo;

    private final PIDController flywheelVelocityController = new PIDController(FLYWHEEL_KP, 0, 0);

    public Outtake(HardwareMap hardwareMap){

        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");

        motorL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorR.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeServo = hardwareMap.servo.get("outtakePitch");

    }

    public void start(double speed){
        motorR.setPower(speed);
        motorL.setPower(speed);
    }

    public void stop(){
        motorR.setPower(0);
        motorL.setPower(0);
    }

    private double startTime = System.nanoTime() / 1e9;
    private int previousTickCount = 0;


    public void setFlywheelSpeed(double goalRPM){

        int currentTickCount = motorL.getCurrentPosition();

        int dtheta = currentTickCount - previousTickCount;
        previousTickCount = currentTickCount;
        double dt = System.nanoTime() / 1e9 - startTime;
        startTime = System.nanoTime() / 1e9;

        double currRPM = dtheta / (dt / 60);

        double wantedWheelPower = flywheelVelocityController.calculate(currRPM, goalRPM);

        motorL.setPower(wantedWheelPower);

    }

    public void startFlywheel(){
       this.startTime = System.nanoTime() / 1e9;  // Current Time in Seconds
    }

    public double currPos(){
        double currPos = (double) (motorR.getCurrentPosition() + motorL.getCurrentPosition()) / 2;
        return currPos;
    }

    public void controlPitch(double angle){
        outtakeServo.setPosition(angle);
    }
}
