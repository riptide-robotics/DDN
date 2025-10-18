package org.firstinspires.ftc.teamcode.Tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Modules.Outtake;

import java.util.LinkedList;
import java.util.Queue;

@Config
@TeleOp(name = "OuttakeTuner",group = "Tuning")
public class OuttakePIDTuner extends LinearOpMode {
    public static double KPTop = 0;
    public static double KPBottom = 0;
    LinkedList<Double> topRecords = new LinkedList<>();
    LinkedList<Double> bottomRecords = new LinkedList<>();
// comment
    Outtake outtake;
    public static double rpmTop = 360;
    public static int queueSize = 5;
    private static double rpmTopPrev = 360;

    public static double rpmBottom = 360;
    private static double rpmBottomPrev = 360;

    Telemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    private double prevPosTop, prevPosBottom, currPosTop, currPosBottom;
    private double startTime = System.nanoTime() / 1e9;
    private PIDController RPMControllerTop = new PIDController(KPTop, 0, 0);
    private PIDController RPMControllerBottom = new PIDController(KPBottom, 0, 0);


    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap);
        telemetry.addData("Robot status", "successfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clear();

        startTime = System.nanoTime() / 1e9;


        while (opModeIsActive()) {

            tele.addData("goalRPMTop", rpmTop);
            tele.addData("goalRPMBottom", rpmBottom);


            pidtunedmotor(tele);
            tele.update();
            if (rpmTopPrev != rpmTop) {
                rpmTopPrev = rpmTop;
                /*
                set:
                    KPTop=0.2
                    KPBottom=0.2
                    rpmTop=300
                    rpmBottom=300
                1:
                    currPRMTop: 0.0
                    currRPMBottom: 2691.987410571929

                2:



                 */




                RPMControllerTop = new PIDController(KPTop, 0, 0);
            }
            if (rpmBottomPrev != rpmBottom) {
                rpmBottomPrev = rpmBottom;
                RPMControllerBottom = new PIDController(KPBottom, 0, 0);
            }
        }
    }
    public void pidtunedmotor(Telemetry telemetry) {

        prevPosTop = currPosTop;
        prevPosBottom = currPosBottom;

        currPosTop = outtake.currPosL();
        currPosBottom = outtake.currPosR();

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

        //double averageTop = topRecords.size() >= queueSize ? (topRecords.get(0)+topRecords.get(1)+topRecords.get(2)+topRecords.get(3)+topRecords.get(4))/5 : currRPMTop;
        //double averageBottom = bottomRecords.size() >= queueSize ? (bottomRecords.get(0)+bottomRecords.get(1)+bottomRecords.get(2)+bottomRecords.get(3)+bottomRecords.get(4))/5 : currRPMBottom;

        telemetry.addData("ready", bottomRecords.size() >= queueSize);
        telemetry.addData("top", averageTop);
        telemetry.addData("bottom", averageBottom);

        double wantedWheelPowerTopAverage = RPMControllerTop.calculate(averageTop, rpmTop);
        double wantedWheelPowerBottomAverage = RPMControllerBottom.calculate(averageBottom, rpmBottom);

        outtake.setFlyWheelPower(wantedWheelPowerTopAverage,wantedWheelPowerBottomAverage);
    }
}