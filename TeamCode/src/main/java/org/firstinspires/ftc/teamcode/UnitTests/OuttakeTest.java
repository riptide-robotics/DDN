package org.firstinspires.ftc.teamcode.UnitTests;

// DO NOT TRANSFER

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "outtakeTest")


public class OuttakeTest extends LinearOpMode {
    Robot robot;

    public static double topGoalSpeed = 0;
    public static double bottomGoalSpeed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        telemetry.addData("Robot status", "succesfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clear();
        telemetry.addData("Robot status", "Started!");
        telemetry.update();

        while (opModeIsActive()) {
            outtake();
        }
    }

    public void outtake() {
        robot.getOuttake().setOuttakeRPM(topGoalSpeed,bottomGoalSpeed);
        robot.getOuttake().runOuttakePID(telemetry);
    }
}
