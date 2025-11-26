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

    public double BTop = 3000;
    /**If it goes well, this should have no effect*/
    public double BBottom = 3000;
    public double BBottomSecondary = 1500;
    public double XTop = 3000;
    public double XBottom = 3000;
    public double YTop = 3000;
    public double YBottom = 3000;






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
    public static boolean hasRun = false;
    private void sequencerTest() {

        robot.s.teleloop(telemetry);
        telemetry.addData("action","Working!");
        if (gamepad1.aWasPressed() && robot.s.actions.isEmpty())
            robot.s.addAction(() -> {
                hasRun = true;
            },1);



//        if (gamepad1.b) {
//
//            robot.s.addAction(() -> {
//                robot.getOuttake().runOuttakePID(BTop,BBottom,telemetry);
//            },1);
//
//            robot.s.addAction(() -> {
//                robot.getOuttake().runOuttakePID(BTop,BBottomSecondary,telemetry);
//            },1);
//        }
//
//
//        if (gamepad1.xWasPressed()) {
//            robot.s.addAction(() -> {
//                robot.getOuttake().runOuttakePID(XTop,XBottom,telemetry);
//            },1);
//            robot.s.addAction(() -> {
//                robot.getOuttake().runOuttakePID(XTop,XBottom,telemetry);
//            },1.5);
//        }
//        if (gamepad1.yWasPressed()) {
//            robot.s.addAction(() -> {
//                robot.getOuttake().runOuttakePID(YTop,YBottom,telemetry);
//            },1);
//            robot.s.addAction(() -> {
//                robot.getIntake().spin(0.5); //james says keep this low to be safe. This has the side effect of removing control from the driver.
//            },1);
//        }
        telemetry.update();
    }
}
