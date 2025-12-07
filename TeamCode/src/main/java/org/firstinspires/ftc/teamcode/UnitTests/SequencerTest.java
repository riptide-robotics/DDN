package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    public boolean hasARun = false;
    public boolean hasBARun = false;
    public boolean hasBBRun = false;
    public int numberXRun = 0;

    public boolean hasALoop = false;
    public boolean hasBALoop = false;
    public boolean hasBBLoop = false;

    public int numberXLoop = 0;
    boolean hasXRun = false;

    private void sequencerTest() {

        robot.s.loop();
        telemetry.addData("actionA",hasARun);
        if (gamepad1.aWasPressed() && robot.s.impulseactions.isEmpty())
            robot.s.AddImpulseAction(() -> {
                hasARun = true;
            },1);



        telemetry.addData("actionBA",hasBARun);
        telemetry.addData("actionBB",hasBBRun);

        if (gamepad1.bWasPressed() && robot.s.impulseactions.isEmpty()) {
            robot.s.AddImpulseAction(() -> {
                hasBARun = true;
            }, 1);
            robot.s.AddImpulseAction(() -> {
                hasBBRun = true;
            },1);
        }
        telemetry.addData("actionXCount",numberXRun);
        if (gamepad1.x) {
            robot.s.AddImpulseAction(() -> {
                numberXRun++;
            }, 3);
        }

        telemetry.addData("loopA", hasALoop);
        if (gamepad2.aWasPressed()) {
            robot.s.addLoopAction(() -> {
                hasALoop = true;
                robot.s.getLoopAction("loopA").killAction = true;
            },1,"loopA");
        }

        telemetry.addData("loopBA", hasBALoop);
        telemetry.addData("loopBB", hasBBLoop);

        if (gamepad2.bWasPressed()) {
            robot.s.addLoopAction(() -> {
                hasBALoop = true;
                robot.s.getLoopAction("loopC").killAction = true;
            },1.5,"loopC");
            robot.s.addLoopAction(() -> {
                hasBBLoop = true;
                robot.s.getLoopAction("loopD").killAction = true;
            },2,"loopD");
        }

        telemetry.addData("loopXCount", numberXLoop);

        if (gamepad2.xWasPressed()) {
            hasXRun = true;
            robot.s.addLoopAction(() -> {
                numberXLoop++;
            },1,"loopE");
        }
        if ((!gamepad2.x) && hasXRun) {
            hasXRun = false;
            robot.s.getLoopAction("loopE").killAction = true;
        }

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
