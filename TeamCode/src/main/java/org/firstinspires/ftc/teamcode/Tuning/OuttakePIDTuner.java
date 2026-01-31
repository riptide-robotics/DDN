package org.firstinspires.ftc.teamcode.Tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.riptideUtil;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.LinkedList;

@Config
@TeleOp(name = "OuttakeTuner",group = "Tuning")
public class OuttakePIDTuner extends LinearOpMode {
    public static double KPTop = 0;
    public static double KPBottom = 0;

    Outtake outtake;
    public static double rpmTop = 360;
    private static double rpmTopPrev = 360;

    public static double rpmBottom = 360;
    private static double rpmBottomPrev = 360;


    Telemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap);
        telemetry.addData("Robot status", "successfully initiated");
        telemetry.update();
        robot  = new Robot((hardwareMap));

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clear();

        while (opModeIsActive()) {

            tele.addData("goalRPMTop", rpmTop);
            tele.addData("goalRPMBottom", rpmBottom);
            tele.addData("KP TOP", KPTop);
            tele.addData("KP BOTTOM", KPBottom);

            tele.update();
            if (rpmTopPrev != rpmTop) {
                rpmTopPrev = rpmTop;
                robot.getOuttake().setFlyWheelTopGoal(rpmTop);
                robot.getOuttake().getRPMControllerTop().setPID(KPTop, 0, 0);
            }

            if (rpmBottomPrev != rpmBottom) {
                rpmBottomPrev = rpmBottom;
                robot.getOuttake().setFlywheelBottomGoal(rpmBottom);
                robot.getOuttake().getRPMControllerBottom().setPID(KPBottom, 0, 0);
            }

            if(gamepad1.a){
               robot.getIntake().bootkick(riptideUtil.BOOT_KICKER_UP);
            }
            else{
               robot.getIntake().bootkick(riptideUtil.BOOT_KICKER_RESTING);
            }

            robot.getOuttake().runOuttakePID(tele);
        }
    }

}