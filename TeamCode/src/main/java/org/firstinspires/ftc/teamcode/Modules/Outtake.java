package org.firstinspires.ftc.teamcode.Modules;

import static org.firstinspires.ftc.teamcode.riptideUtil.FLYWHEEL_KP;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.PIDController;

public class Outtake {
    private final DcMotor topFlywheel;
    private final DcMotor bottomFlywheel;

    private final PIDController flywheelVelocityControllerR = new PIDController(FLYWHEEL_KP, 0, 0);
    private final PIDController flywheelVelocityControllerL = new PIDController(FLYWHEEL_KP, 0, 0);

    public Outtake(HardwareMap hardwareMap){

        topFlywheel = hardwareMap.dcMotor.get("topFlywheel");
        bottomFlywheel = hardwareMap.dcMotor.get("bottomFlywheel");

        topFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void start(double speed){
        bottomFlywheel.setPower(speed);
        topFlywheel.setPower(speed);
    }

    //This one method is only to set motors individually
    public void setFlyWheelPower(double speedL, double speedR) {
        bottomFlywheel.setPower(speedR);
        topFlywheel.setPower(speedL);

    }

    public void stop(){
        bottomFlywheel.setPower(0);
        topFlywheel.setPower(0);
    }

    private double startTime = System.nanoTime() / 1e9;
    private int previousTickCountL = 0;
    private int previousTickCountR = 0;


    public void setFlywheelSpeed(double goalRPM){

        int currentTickCountL = topFlywheel.getCurrentPosition();
        int currentTickCountR = bottomFlywheel.getCurrentPosition();

        double dthetaL = (currentTickCountL - previousTickCountL)/28D;
        previousTickCountL = currentTickCountL;

        double dthetaR = (currentTickCountR - previousTickCountR)/28D;
        previousTickCountR = currentTickCountR;

        double dt = System.nanoTime() / 1e9 - startTime;
        startTime = System.nanoTime() / 1e9;



        double currRPML = dthetaL / (dt / 60);
        double currRPMR = dthetaR / (dt / 60);

        double wantedWheelPowerR = flywheelVelocityControllerR.calculate(currRPMR, goalRPM);
        double wantedWheelPowerL = flywheelVelocityControllerL.calculate(currRPML, goalRPM);

        topFlywheel.setPower(wantedWheelPowerL);
        bottomFlywheel.setPower(wantedWheelPowerR);

    }

    public void startFlywheel(){
        this.startTime = System.nanoTime() / 1e9;  // Current Time in Seconds
    }

    public double currPos(){
        double currPos = (double) (bottomFlywheel.getCurrentPosition() + topFlywheel.getCurrentPosition()) / 2;
        return currPos;
    }

    //Created to allow OuttakePIDTuner to access individual positions
    public double currPosL(){
        return topFlywheel.getCurrentPosition();
    }
    public double currPosR(){
        return bottomFlywheel.getCurrentPosition();
    }
    //Stops here



}
