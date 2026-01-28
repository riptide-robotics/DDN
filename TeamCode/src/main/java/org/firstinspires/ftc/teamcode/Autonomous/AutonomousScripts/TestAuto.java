package org.firstinspires.ftc.teamcode.Autonomous.AutonomousScripts;

import androidx.xr.runtime.math.Pose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Trajectories.LinearTrajectoryBuilder;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous
public class TestAuto extends LinearOpMode {
    Robot robot;
    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    Trajectory test = new LinearTrajectoryBuilder()
            .moveTo(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0))
            .moveTo(new Pose2D(DistanceUnit.INCH, 0, 48, AngleUnit.DEGREES, 0))
            .moveTo(new Pose2D(DistanceUnit.INCH, 48, 48, AngleUnit.DEGREES, 0))
            .moveTo(new Pose2D(DistanceUnit.INCH, 48, 0, AngleUnit.DEGREES, 0))
            .moveTo(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0))
            .segmentSpeedScalar(0, 0.5)
            .segmentSpeedScalar(1, 0.5)
            .segmentSpeedScalar(2, 0.5)
            .segmentSpeedScalar(3, 0.5)
            .segmentSpeedScalar(4, 0.5)
            .build();

    public static boolean start = false;
    boolean debounce = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        ElapsedTime e = new ElapsedTime();

        waitForStart();
        if (isStopRequested()) return;

        e.reset();

        while (opModeIsActive()) {

            Trajectory.PathSample expectedPos = test.getExpectedPosition(0);

            if (start) {
                if (!debounce) {
                    e.reset();
                    debounce = true;
                }

                expectedPos = test.getExpectedPosition(e.time(TimeUnit.SECONDS));

                Pose2D expectedPosButInPose2D = new Pose2D(DistanceUnit.INCH, expectedPos.x, expectedPos.y, AngleUnit.DEGREES, expectedPos.heading);

                robot.getDrivetrain().goToPosPID(expectedPosButInPose2D);
            }


            t.addData("expected x", expectedPos.x);
            t.addData("expected y", expectedPos.y);
            t.addData("actual x", robot.getDrivetrain().getCurrPos().getX(DistanceUnit.INCH));
            t.addData("actual y", robot.getDrivetrain().getCurrPos().getY(DistanceUnit.INCH));
            t.update();
        }


    }
}
