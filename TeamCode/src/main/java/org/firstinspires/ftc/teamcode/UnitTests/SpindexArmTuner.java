package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "spindex tuner")
public class SpindexArmTuner extends LinearOpMode {
    Robot robot;

    public static double up = 0.1;
    public static double resting = 0.5;

    public static double spindexPosfull = 0;
    public static  double spindexPosNormal = 0;

    public static double topgoal = 0;
    public static  double bottomgoal = 0;
    Telemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws  InterruptedException{
        robot = new Robot(hardwareMap);


        robot.getOuttake().startFlywheel();



        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.x){
                robot.getIntake().BootKick(up);
            } else {
                robot.getIntake().closeTransfer(resting);
            }

            if (gamepad2.y){
                robot.getIntake().spindexPos2to1Gear(spindexPosfull);
            } else{
                robot.getIntake().spindexPos(spindexPosNormal);
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
