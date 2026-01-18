package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "spindex arm tuner")
public class SpindexArmTuner extends LinearOpMode {
    Robot robot;

    public static double up = 0.2;
    public static double resting = 1;

    public static double spindexPosfull = 0;
    public static  double spindexPosNormal = 0;

    public static double topgoal = 0;
    public static  double bottomgoal = 0;
    Telemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws  InterruptedException{
        robot = new Robot(hardwareMap);
        robot.getIntake().initSpindex();


        robot.getOuttake().startFlywheel();



        waitForStart();
        while (opModeIsActive()){
            robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_0_SHOOT);
            if (gamepad2.x){
                robot.getIntake().bootkick(up);
            } else {
                robot.getIntake().bootkick(resting);
            }


            if (gamepad1.b){
                topgoal = 3000;
                bottomgoal = 3000;
            } else{
                topgoal = 0;
                bottomgoal = 0;
            }

            robot.getOuttake().runOuttakePID(topgoal, bottomgoal, tele);
        }
    }
}
