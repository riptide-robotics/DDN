package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.riptideUtil.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Auto Shoot Three")
public class AutoShootThree extends LinearOpMode {
    Robot robot;
    double slot = 0;

    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        robot.getIntake().initSpindex();
        robot.getIntake().setSlotColors('p','p','p');

        t.addData("Robot status:", "succesfully initiated");
        t.update();

        ElapsedTime e = new ElapsedTime();
        e.reset();

        waitForStart();
        if (isStopRequested()) return;

        robot.s.addLoopAction(() -> {
            robot.getDrivetrain().setWheelPowers(1, 1, 1, 1);
        }, 0,"Drive Forward");

        robot.s.addImpulseAction(() -> {
            robot.s.killLoopAction("Drive Forward", true);
        },0.75);

        robot.s.addLoopAction(() -> {
            robot.getDrivetrain().setWheelPowers(0, 0, 0, 0);
        }, 0.75,"Stop");

        robot.s.addImpulseAction(() -> {
            robot.s.killLoopAction("Stop", true);
        },3);

        robot.s.addLoopAction(() -> {
            robot.getDrivetrain().turnOnPointPID(-90);
        }, 3,"Turn on Point");

        robot.s.addImpulseAction(() -> {
            robot.s.killLoopAction("Turn on Point", true);
        },6);

        // Shoot

        robot.s.addLoopAction(() -> {
            robot.getOuttake().setOuttakeRPM(MID_DIST_TOP, MID_DIST_BOT);
            robot.getOuttake().runOuttakePID(t);
        }, 6, "Shoot");

        robot.s.addImpulseAction(() -> {
            slot = robot.getIntake().getNextOuttakeSlot();
            robot.getIntake().cycleOuttakeSlot(slot);
        },10);

        robot.s.addImpulseAction(() -> {
            robot.outtake(slot, t);
        },11);

        robot.s.addImpulseAction(() -> {
            slot = robot.getIntake().getNextOuttakeSlot();
            robot.getIntake().cycleOuttakeSlot(slot);
        },14);

        robot.s.addImpulseAction(() -> {
            robot.outtake(slot, t);
        },15);

        robot.s.addImpulseAction(() -> {
            slot = robot.getIntake().getNextOuttakeSlot();
            robot.getIntake().cycleOuttakeSlot(slot);
        },18);

        robot.s.addImpulseAction(() -> {
            robot.outtake(slot, t);
        },19);

        robot.s.addImpulseAction(() -> {
            robot.getIntake().bootkick(BOOT_KICKER_RESTING);
        },20);

        while (opModeIsActive()) {
            robot.s.loop();

            t.update();
        }
    }
}
