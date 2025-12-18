package org.firstinspires.ftc.teamcode.Modules;

import static org.firstinspires.ftc.teamcode.riptideUtil.KPBottom;
import static org.firstinspires.ftc.teamcode.riptideUtil.KPTop;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURNTABLE_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURNTABLE_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURNTABLE_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.angularVelocity;
import static org.firstinspires.ftc.teamcode.riptideUtil.econserved;
import static org.firstinspires.ftc.teamcode.riptideUtil.tolerance;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Placeholder;

import java.util.LinkedList;

public class Outtake {
    private final DcMotor topFlywheel;
    private final DcMotor bottomFlywheel;

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
    private boolean atGoalSpeed = false;
    private double rpmTopGoal = 0;
    private double rpmBottomGoal = 0;

    //Turntable

    private double turntableAngle = 0;
    private PIDController turntableController = new PIDController(TURNTABLE_KP, TURNTABLE_KI, TURNTABLE_KD);
    static final double ticksToDegrees = 360/751.8;
    static final double degreesToTicks = 751.8/360;
    private final DcMotor turntable;

    public Outtake(HardwareMap hardwareMap){
        topFlywheel = hardwareMap.dcMotor.get("topFlywheel");
        bottomFlywheel = hardwareMap.dcMotor.get("bottomFlywheel");

        topFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        turntable = hardwareMap.dcMotor.get("turnTable");
        turntable.setDirection(DcMotorSimple.Direction.FORWARD);
        turntable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Prepares timer. Run this just before using anything else.
     **/
    public void startFlywheel(){
        this.startTime = System.nanoTime() / 1e9;  // Current Time in Seconds
    }


    /**
     * Recommended method of setting the outtake. Sets it moving.
     **/
    public void runOuttakePID(double rpmTop, double rpmBottom, Telemetry tele){

        pidtunedmotor(rpmTop, rpmBottom, tele);

        tele.addData("goalRPMTop", rpmTop);
        tele.addData("goalRPMBottom", rpmBottom);

        if (rpmTopPrev != rpmTop) {
            rpmTopPrev = rpmTop;
            RPMControllerTop = new PIDController(KPTop, 0, 0);

        }
        if (rpmBottomPrev != rpmBottom) {
            rpmBottomPrev = rpmBottom;
            RPMControllerBottom = new PIDController(KPBottom, 0, 0);
        }
    }


    /**
     * Runs based on stored goal instead of target RPMs given by the user.
     **/
    public void runOuttakePID(Telemetry tele){runOuttakePID(rpmTopGoal, rpmBottomGoal, tele);}


    /**
     * Similar to runOuttakePID, but doesn't handle change as well. Not recommended.
     **/
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

//        averageTop = topRecords.size() >= queueSize ? (topRecords.get(0)+topRecords.get(1)+topRecords.get(2)+topRecords.get(3)+topRecords.get(4))/5 : currRPMTop;
//        averageBottom = bottomRecords.size() >= queueSize ? (bottomRecords.get(0)+bottomRecords.get(1)+bottomRecords.get(2)+bottomRecords.get(3)+bottomRecords.get(4))/5 : currRPMBottom;


        double wantedWheelPowerTopAverage = RPMControllerTop.calculate(averageTop-150, rpmTop);
        double wantedWheelPowerBottomAverage = RPMControllerBottom.calculate(averageBottom-150, rpmBottom);


        setFlyWheelPower(rpmTop != 0 ? wantedWheelPowerTopAverage:0,rpmBottom != 0 ? wantedWheelPowerBottomAverage:0);


        telemetry.addData("ready", bottomRecords.size() >= queueSize);
        telemetry.addData("top", averageTop);
        telemetry.addData("bottom", averageBottom);


        boolean atTopRPM = Math.abs(averageTop - rpmTop) <= tolerance;
        boolean atBotRPM = Math.abs(averageBottom - rpmBottom) <= tolerance;

        atGoalSpeed = atTopRPM && atBotRPM;
    }


    /**
     * Sets the motors based upon distance from the goal.
     **/
    @SuppressLint("DefaultLocale")
    public void setPowerOnDist(Double dist /* INCHES */, boolean run, Telemetry tele) {

        double rBottom = 1.41732; // INCHES
        double rTop = 1.41732; // INCHES
        double angle = Math.toRadians(75);
        double y = 25 - rBottom;

        double term = dist * Math.tan(angle) - y;
        if (term <= 0) {
            tele.addData("term", term);
            pidtunedmotor(0, 0, tele);
            return;
        }

        double numerator = 9.8 * 39.3701 * dist * dist;
        double denominator = 2 * Math.pow(Math.cos(angle), 2) * term;

        double ballVelocity = Math.sqrt(numerator / denominator);

        double rpmTop;
        double rpmBottom;

        if (run) {
            rpmTop = ((ballVelocity * 60) / (2 * Math.PI * rTop)) * econserved - (angularVelocity * 30 / Math.PI) * econserved;
            rpmBottom = ((ballVelocity * 60) / (2 * Math.PI * rBottom)) * econserved - (angularVelocity * 30 / Math.PI) * econserved;
        } else {
            rpmTop = 0;
            rpmBottom = 0;
        }

        pidtunedmotor(rpmTop, rpmBottom, tele);

        tele.addLine(String.format("Calculated rpm top: %f", rpmTop));
        tele.addLine(String.format("Calculated rpm bottom: %f", rpmBottom));
    }


    /**
     * Places goals instead of directly commanding motors. <br>
     * "Now it's some other poor soul's job!" - this method
     **/
    public void setOuttakeRPM(double rpmTop, double rpmBottom){rpmTopGoal = rpmTop; rpmBottomGoal = rpmBottom;}

    /**
     * Sets the the goal for the top motor to reach in its PID. For the future.
     **/
    public void SetFlyWheelTopGoal(double rpm) {rpmTopGoal = rpm;}


    /** Sets the the goal for the bottom motor to reach in its PID. For the future.
     **/
    public void setFlywheelBottomGoal(double rpm) {rpmBottomGoal = rpm;}


    /**
     * Completely skip tuning and run both motors at the same voltage.
     **/
    public void setFlywheelPowersNoPID(double speed){
        bottomFlywheel.setPower(speed);
        topFlywheel.setPower(speed);
    }


    /**Completely skip tuning and set motor voltages individually. As fundamental as you get.
     * */
    public void setFlyWheelPower(double speedT, double speedB) {
        bottomFlywheel.setPower(speedB);
        topFlywheel.setPower(speedT);
    }


    /**
     * Directly stop power to both motors. Works surprisingly well, although not really used.
     **/
    public void stop(){
        bottomFlywheel.setPower(0);
        topFlywheel.setPower(0);
    }


    /**
     * Checks if it is within acceptable ranges.
     **/
    public boolean isAtGoalSpeed(){
        return atGoalSpeed;
    }


    /**
     *Gets the current position, in ticks, of the top flywheel.
     **/
    public double currPosL(){
        return topFlywheel.getCurrentPosition();
    }


    /**
     * Gets the current position, in ticks, of the bottom flywheel.
     **/
    public double currPosR(){
        return bottomFlywheel.getCurrentPosition();
    }

    /**
     * Manual override. Maps a joystick input [-1, 1] to a (-90, 90) degree PID call
     * @param joy a double in the range of [-1, 1].
     * @return nothing
     */
    public void mapJoyToAngle(double joy){
        joy = Math.max(-1, Math.min(1, joy));
        double angle = 90 * joy;
        setTurntableGoalAngle(angle);

    }

    /**
     * sets some internal variable that has an angle MAKE SURE THAT ANGLES ARE BOUNDED. Also not working just yet.
     **/
    public void setTurntableGoalAngle(double angle){
        turntableAngle = angle;
    }

    /**
     * Pid Controller for the turntable
     **/
    public void updateTurntableAngle(Telemetry t) {
        double currPos = turntable.getCurrentPosition();
        double goalAngleInTicks = turntableAngle * 3 * degreesToTicks;
        double power = turntableController.calculate(currPos, goalAngleInTicks);
        turntable.setPower(power);
        t.addData("Goal Angle", turntableAngle);
        t.addData("Turntable angle", currPos * ticksToDegrees / 3);
    }


}
