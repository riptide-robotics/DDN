package org.firstinspires.ftc.teamcode.Modules;

import static org.firstinspires.ftc.teamcode.riptideUtil.TOP_FLYWHEEL_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.BOTTOM_FLYWHEEL_KP;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.PIDController;

import java.util.LinkedList;

public class Outtake {
    private final DcMotor topFlywheel;
    private final DcMotor bottomFlywheel;

    private final PIDController flywheelVelocityControllerBottom = new PIDController(TOP_FLYWHEEL_KP, 0, 0);
    private final PIDController flywheelVelocityControllerTop = new PIDController(BOTTOM_FLYWHEEL_KP, 0, 0);

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

    LinkedList<Double> topRecords = new LinkedList<>();
    LinkedList<Double> bottomRecords = new LinkedList<>();

    private double prevPosTop, prevPosBottom, currPosTop, currPosBottom;

    public static int queueSize = 5;

    public void setFlywheelSpeed(double goalRPMTop, double goalRPMBottom){

        prevPosTop = currPosTop;
        prevPosBottom = currPosBottom;

        currPosTop = currPosL();
        currPosBottom = currPosR();

        double dThetaTop = (currPosTop - prevPosTop)/28;
        double dThetaBottom = (currPosBottom - prevPosBottom)/28;

        double dt = System.nanoTime() / 1e9 - startTime;
        startTime = System.nanoTime() / 1e9;

        double currRPMTop = dThetaTop / (dt / 60);
        double currRPMBottom = dThetaBottom / (dt / 60);

        // telemetry.addData("currPRMTop", currRPMTop);
        // telemetry.addData("currRPMBottom", currRPMBottom);

        //double wantedWheelPowerTop = RPMControllerTop.calculate(currRPMTop, rpmTop);
        //double wantedWheelPowerBottom = RPMControllerBottom.calculate(currRPMBottom, rpmBottom);

        topRecords.add(currRPMTop);
        if (topRecords.size() > queueSize)
            topRecords.remove(0);

        bottomRecords.add(currRPMBottom);
        if (bottomRecords.size() > queueSize)
            bottomRecords.remove(0);

        double undividedAverageBottom = 0;
        double undividedAverageTop = 0;

        for (int i = 0; i < topRecords.size(); i++) {
            undividedAverageTop += topRecords.get(i);
            undividedAverageBottom += topRecords.get(i);
        }
        double averageTop;
        double averageBottom;
        if (undividedAverageTop > 0) {
            averageTop = undividedAverageTop / queueSize;
            averageBottom = undividedAverageBottom / queueSize;
        } else {
            averageTop = currRPMTop;
            averageBottom = currRPMBottom;
        }

        double wantedWheelPowerTopAverage = flywheelVelocityControllerTop.calculate(averageTop, goalRPMTop);
        double wantedWheelPowerBottomAverage = flywheelVelocityControllerBottom.calculate(averageBottom, goalRPMBottom);

        setFlyWheelPower(wantedWheelPowerTopAverage,wantedWheelPowerBottomAverage);
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
