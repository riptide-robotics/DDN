package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "SequencerTripTest")
public class SequencerTripTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;

        robot.getOuttake().startFlywheel();

        while (opModeIsActive()) {
            sequencerTripTest();
        }
    }

    private void sequencerTripTest() {
    }
}
