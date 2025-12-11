package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Indicator;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.teamcode.Modules.Utils.Sequencer;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "SequencerTripTest")
public class SequencerTripTest extends LinearOpMode {
    Robot robot;

    Sequencer.Area area;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        area = new Sequencer.Area(1,3,4,6);

        waitForStart();
        if (isStopRequested()) return;

        robot.getOuttake().startFlywheel();

        while (opModeIsActive()) {
            sequencerTripTest();
            telemetry.update();
        }
    }

    boolean impulseRun = false;
    int loopCount = 0;
    private void sequencerTripTest() {
        telemetry.addData("TImpulseTest Run",impulseRun);
        telemetry.addData("TLoopTest Loop Count", loopCount);
        telemetry.addData("Within Range", area.botWithinArea(robot.getDrivetrain()));
        telemetry.addData("Current X", robot.getDrivetrain().getPinpoint().getPosX(Sequencer.unit));



        if (gamepad1.x) {
            robot.s.addTImpulseAction(() -> {
                impulseRun = true;
            },area,"TImpulseTest");
        }
        if (gamepad1.y) {
            robot.s.addTImpulseAction(() -> {
                robot.getOuttake().runOuttakePID(3000,3000,telemetry);
                loopCount++;
            },area,"TLoopTest");
        } else {
            robot.getOuttake().runOuttakePID(0,0,telemetry);
        }

        if (gamepad1.aWasPressed()) {
            robot.getDrivetrain().setWheelPowers(0.3,0.3,0.3,0.3);
        }
        if (gamepad1.bWasPressed()) {
            robot.getDrivetrain().setWheelPowers(-0.3,-0.3,-0.3,-0.3);
        }
    }
}
