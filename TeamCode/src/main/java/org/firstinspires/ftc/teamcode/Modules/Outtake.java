package org.firstinspires.ftc.teamcode.Modules;

import static org.firstinspires.ftc.teamcode.riptideUtil.DEADZONE;
import static org.firstinspires.ftc.teamcode.riptideUtil.DEGREES_TO_TICKS;
import static org.firstinspires.ftc.teamcode.riptideUtil.KPBottom;
import static org.firstinspires.ftc.teamcode.riptideUtil.KPTop;
import static org.firstinspires.ftc.teamcode.riptideUtil.TICKS_TO_DEGREES;
import static org.firstinspires.ftc.teamcode.riptideUtil.TOP_FLYWHEEL_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.BOTTOM_FLYWHEEL_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURNTABLE_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURNTABLE_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURNTABLE_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.angularVelocity;
import static org.firstinspires.ftc.teamcode.riptideUtil.econserved;
import static org.firstinspires.ftc.teamcode.riptideUtil.tolerance;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedList;

public class Outtake {
    private final DcMotor topFlywheel;
    private final DcMotor bottomFlywheel;

    private final PIDController flywheelVelocityControllerBottom = new PIDController(TOP_FLYWHEEL_KP, 0, 0);
    private final PIDController flywheelVelocityControllerTop = new PIDController(BOTTOM_FLYWHEEL_KP, 0, 0);

    private boolean updatePID = false;

    // turntable vars
    private final DcMotor turntable;

    private final PIDController turntablePid = new PIDController(TURNTABLE_KP, TURNTABLE_KI, TURNTABLE_KD);

    private double turntableAbsoluteAngle;
    private double turntableGoalAngle;
    private double turntablePrevGoalAngle;
    private double turntableGoalTicks;

    // OUTTAKE TUNER THINGS
    LinkedList<Double> topRecords = new LinkedList<>();
    LinkedList<Double> bottomRecords = new LinkedList<>();
    public static int queueSize = 3;
    private static double rpmTopPrev = 360;
    private static double rpmBottomPrev = 360;

    private double currPosTop;
    private double currPosBottom;
    private double startTime = System.nanoTime() / 1e9;
    private PIDController RPMControllerTop = new PIDController(KPTop, 0, 0);
    private PIDController RPMControllerBottom = new PIDController(KPBottom, 0, 0);
    private double prevPosTop = 0;
    private  double prevPosBottom = 0;
    public void runOuttakePID(double rpmTop, double rpmBottom, Telemetry tele){

        pidtunedmotor(rpmTop, rpmBottom, tele);

        tele.addData("goalRPMTop", rpmTop);
        tele.addData("goalRPMBottom", rpmBottom);
        tele.update();
        if (rpmTopPrev != rpmTop) {
            rpmTopPrev = rpmTop;
            RPMControllerTop = new PIDController(KPTop, 0, 0);

        }
        if (rpmBottomPrev != rpmBottom) {
            rpmBottomPrev = rpmBottom;
            RPMControllerBottom = new PIDController(KPBottom, 0, 0);
        }
        tele.update();
    }

    private boolean atGoalSpeed = false;


    public void pidtunedmotor(double rpmTop, double rpmBottom, Telemetry telemetry) {

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

        topRecords.add(currRPMTop);
        while (topRecords.size() > queueSize)
            topRecords.remove(0);

        bottomRecords.add(currRPMBottom);
        while (bottomRecords.size() > queueSize)
            bottomRecords.remove(0);

        double undividedAverageBottom = 0;
        double undividedAverageTop = 0;

        for (int i = 0; i < topRecords.size(); i++) {
            undividedAverageTop += topRecords.get(i);
            undividedAverageBottom += bottomRecords.get(i);
        }
        double averageTop;
        double averageBottom;
        if (topRecords.size() == queueSize) {
            averageTop = undividedAverageTop / queueSize;
            averageBottom = undividedAverageBottom / queueSize;
        } else {
            averageTop = currRPMTop;
            averageBottom = currRPMBottom;
        }

        //double averageTop = topRecords.size() >= queueSize ? (topRecords.get(0)+topRecords.get(1)+topRecords.get(2)+topRecords.get(3)+topRecords.get(4))/5 : currRPMTop;
        //double averageBottom = bottomRecords.size() >= queueSize ? (bottomRecords.get(0)+bottomRecords.get(1)+bottomRecords.get(2)+bottomRecords.get(3)+bottomRecords.get(4))/5 : currRPMBottom;

        telemetry.addData("ready", bottomRecords.size() >= queueSize);
        telemetry.addData("top", averageTop);
        telemetry.addData("bottom", averageBottom);

        double wantedWheelPowerTopAverage = RPMControllerTop.calculate(averageTop - 200, rpmTop);
        double wantedWheelPowerBottomAverage = RPMControllerBottom.calculate(averageBottom - 200, rpmBottom);


        setFlyWheelPower(rpmTop != 0 ? wantedWheelPowerTopAverage:0,rpmBottom != 0 ? wantedWheelPowerBottomAverage:0);

        telemetry.addData("ready", bottomRecords.size() >= queueSize);
        telemetry.addData("top", averageTop);
        telemetry.addData("bottom", averageBottom);


        boolean atTopRPM = Math.abs(averageTop - rpmTop) <= tolerance;
        boolean atBotRPM = Math.abs(averageBottom - rpmBottom) <= tolerance;
        atGoalSpeed = atTopRPM && atBotRPM;
    }

    public Outtake(HardwareMap hardwareMap){

        topFlywheel = hardwareMap.dcMotor.get("topFlywheel");
        bottomFlywheel = hardwareMap.dcMotor.get("bottomFlywheel");

        topFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        turntable = hardwareMap.dcMotor.get("turntable");
        turntable.setDirection(DcMotor.Direction.FORWARD);
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turntable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turntablePid.setDeadZone(DEADZONE);
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


    public boolean isAtGoalSpeed(){
        return atGoalSpeed;
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


    public void setPowerOnDist(double dist /* INCHES */, boolean run, Telemetry tele){
        double rBottom = 1.41732; // INCHES
        double rTop = 1.41732; // INCHES
        double angle = Math.toRadians(30);
        double height = (23 /* height of artifact in outtake */- rBottom - Math.cos(angle) * (rBottom + 2.4 * Math.sqrt(2) - 1));

        double rpmTop;
        double rpmBottom;

        double numerator = (9.8 * 39.3701 /* meters per second to inches per second*/) * Math.pow(dist, 2);
        double denominator = 2 * Math.pow(Math.cos(angle), 2) * (dist * Math.tan(angle) - height);

        double ballVelocity = Math.sqrt(numerator / denominator);

        if (run) {
            rpmTop = ((ballVelocity * 60) / (2 * Math.PI * rTop)) * econserved - (angularVelocity * 30 / Math.PI) * econserved;
            rpmBottom = ((ballVelocity * 60) / (2 * Math.PI * rBottom)) * econserved - (angularVelocity * 30 / Math.PI) * econserved;
        }
        else {rpmTop = 0; rpmBottom = 0;}
        pidtunedmotor(rpmTop, rpmBottom, tele);
        tele.addData("Calculated rpm top: ", rpmTop);
        tele.addData("Calculated rpm bottom: ", rpmBottom);
    }

    private double rpmTopGoal;
    private double rpmBottomGoal;

    public void setOuttakeRPM (double rpmTop, double rpmBottom){rpmTopGoal = rpmTop; rpmBottomGoal = rpmBottom;}
    public void runOuttakePID(Telemetry tele){pidtunedmotor(rpmTopGoal, rpmBottomGoal, tele);}

    // turntable stuff
    public void setTurntableGoalAngle (double goalAngle) {
        turntableGoalAngle = goalAngle;
    }

    public void setGoalWithError (double error) {
        double turntableAngle = turntable.getCurrentPosition() * TICKS_TO_DEGREES;
        if (error + turntableAngle < 0) {
            turntableAbsoluteAngle = error + turntableAngle + 360;
        }
    }

    public ElapsedTime turntableTime= new ElapsedTime();
    public void turntableGoToAngle() {
        if(turntablePrevGoalAngle != (turntableGoalAngle * 3)) {
            turntableTime.reset();
            turntablePrevGoalAngle = (turntableGoalAngle * 3);
        }
        turntablePid.setPID(TURNTABLE_KP, TURNTABLE_KI, TURNTABLE_KD);

        turntableGoalTicks = (turntableGoalAngle * 3) * DEGREES_TO_TICKS;

        double currPosTicks = turntable.getCurrentPosition();
        turntable.setPower(turntablePid.calculate(currPosTicks, turntableGoalTicks));
    }
}
