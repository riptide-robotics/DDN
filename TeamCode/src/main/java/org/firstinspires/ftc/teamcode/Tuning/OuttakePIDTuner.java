package org.firstinspires.ftc.teamcode.Tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.DummyClasses.Outtake;

@Config
@TeleOp(name = "OuttakeTuner",group = "Tuning")
public class OuttakePIDTuner extends LinearOpMode {
    public static double KP = 0;
    Outtake outtake;
    public static double rpmTop = 360;
    public static double rpmBottom = 360;
    Telemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



    private double prevPosTop, prevPosBottom, currPosTop, currPosBottom;
    private double startTime = System.nanoTime() / 1e9;
    private final PIDController RPMControllerTop = new PIDController(KP, 0, 0);
    private final PIDController RPMControllerBottom = new PIDController(KP, 0, 0);


    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap);
        telemetry.addData("Robot status", "successfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clear();

        while (opModeIsActive()) {
            pidtunedmotor(tele);
            tele.update();
        }
    }
    public void pidtunedmotor(Telemetry telemetry) {

        startTime = System.nanoTime() / 1e9;
        
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

        telemetry.addData("Current RPM of top outtake motor", currRPMTop);
        telemetry.addData("Current RPM of bottom outtake motor", currRPMBottom);

        double wantedWheelPowerTop = RPMControllerTop.calculate(currRPMTop, rpmTop);
        double wantedWheelPowerBottom = RPMControllerBottom.calculate(currRPMBottom, rpmBottom);

        outtake.setFlyWheelPower(wantedWheelPowerTop,wantedWheelPowerBottom);
    }

}