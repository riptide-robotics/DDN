package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "spindex tuner")
public class SpindexArmTuner extends LinearOpMode {
    Robot robot;

    public static double up = 0;
    public static double resting = 0.8;

    public static double spindexPos = 0;

    @Override
    public void runOpMode() throws  InterruptedException{
        robot = new Robot(hardwareMap);

        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.x){
                robot.getIntake().BootKick(up);
            } else {
                robot.getIntake().closeTransfer(resting);
            }

            if (gamepad1.y){
                robot.getIntake().spindexPos(robot.getIntake().fiveTurnToServo(spindexPos));
            } else {
                robot.getIntake().spindexPos(0);
            }
        }
    }
}
