package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Sequencer;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "SequencerTest")
public class SequencerTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        robot.getOuttake().startFlywheel();

        while (opModeIsActive()) {
            sequencerTest();
        }
    }
    private void sequencerTest() {

        robot.s.loop();

        if (gamepad1.aWasPressed())
            robot.s.addAction(() -> {
                telemetry.addData("action A","Working!");
            },1);


        if (gamepad1.b) {

            robot.s.addAction(() -> {
                robot.getOuttake().runOuttakePID(3000,3000,telemetry);
            },1);

            robot.s.addAction(() -> {
                robot.getOuttake().runOuttakePID(3000,1500,telemetry);
            },1);
        }


        if (gamepad1.xWasPressed()) {
            robot.s.addAction(() -> {
                robot.getOuttake().runOuttakePID(3000,3000,telemetry);
            },1);
            robot.s.addAction(() -> {
                robot.getOuttake().runOuttakePID(3000,3000,telemetry);
            },1.5);
        }
        if (gamepad1.yWasPressed()) {
            robot.s.addAction(() -> {
                robot.getOuttake().runOuttakePID(3000,3000,telemetry);
            },1);
            robot.s.addAction(() -> {
                robot.getIntake().spin(0.5);
            },1);
        }
    }
}
