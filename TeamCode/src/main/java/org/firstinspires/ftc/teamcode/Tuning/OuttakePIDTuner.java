package org.firstinspires.ftc.teamcode.Tuning;

import static org.firstinspires.ftc.teamcode.riptideUtil.FLYWHEEL_KP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.DummyClasses.Outtake;

@Config
@TeleOp(name = "OuttakeTuner",group = "Tuning")
public class OuttakePIDTuner extends LinearOpMode {
    public static double KP = 0;
    Outtake outtake;
    public static double rpmL = 360;
    public static double rpmR = 360;

    public static boolean recordspeed = false;
    public static boolean repeatrecord = false;


    Telemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    ElapsedTime t = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap);
        telemetry.addData("Robot status", "successfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clear();

        preparefirstframe();

        while (opModeIsActive()) {
//            outtake.startFlywheel();
//            outtake.setFlywheelSpeed(rpm);
//            //saveframe();



            if (recordspeed)
   //             telemetry.addData("Outtake speed", motorvelocity(frame1dist,frame2dist, frame1time, frame2time));

                tele.addData("Speed of motor",speed());
        }
    }

    double frame1dist =-1;
    double frame2dist =-1;
    double frame1time =-1;
    double frame2time =-1;
//    public void saveframe() {
//        frame2time = frame1time;
//        frame2dist = frame1dist;
//        frame1time = t.nanoseconds();
//        frame1dist = outtake.currPos();
//    }
//
//
//
//    public double getouttakerpm() {
//
//
//
//
//        return -1;
//    }
    public double motorvelocity(double dist1, double dist2, double frame1, double frame2) {
        if (!repeatrecord)
            recordspeed = false;
        return (dist2-dist1)/(frame2-frame1);
    }
//
//
    public void preparefirstframe() {
        frame1dist = outtake.currPos();
        frame1time = t.nanoseconds();
    };

    public double speed() {
        frame2time = frame1time;
        frame2dist = frame1dist;

        preparefirstframe();

        return motorvelocity(frame1dist,frame2dist,frame1time,frame2time);


    }

    private double prevPosL, prevPosR, currPosL, currPosR;

    private double startTime = System.nanoTime() / 1e9;

    private final PIDController RPMControllerL = new PIDController(KP, 0, 0);

    private final PIDController RPMControllerR = new PIDController(KP, 0, 0);

    
    public void pidtunedmotor() {
        startTime = System.nanoTime() / 1e9;
        
        prevPosL = currPosL;
        prevPosR = currPosR;
        
        currPosL = outtake.currPosL();
        currPosR = outtake.currPosR();
        
        double dThetaL = (currPosL - prevPosL)/28;
        double dThetaR = (currPosR - prevPosR)/28;

        double dt = System.nanoTime() / 1e9 - startTime;
        startTime = System.nanoTime() / 1e9;
        
        double currRPML = dThetaL / (dt / 60);
        double currRPMR = dThetaR / (dt / 60);

        double wantedWheelPowerL = RPMControllerL.calculate(currRPMR, rpmL);
        double wantedWheelPowerR = RPMControllerR.calculate(currRPML, rpmR);

        outtake.setFlyWheelPower(wantedWheelPowerL,wantedWheelPowerR);
    }

}