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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "Auto Shoot Three")
public class AutoShootThree extends LinearOpMode {
    Robot robot;
    double slot = 0;
    boolean isRed = false;

    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        robot.getIntake().initSpindex();
        robot.getIntake().setSlotColors('g','p','p');

        t.addData("Robot status:", "succesfully initiated");
        t.update();

        ElapsedTime e = new ElapsedTime();
        e.reset();

        waitForStart();
        if (isStopRequested()) return;

        robot.s.addLoopAction(() -> {
            robot.getDrivetrain().setWheelPowers(-1, -1, -1, -1);
        }, 0,"Drive Forward");

        robot.s.addImpulseAction(() -> {
            robot.s.killLoopAction("Drive Forward", true);
        },1.3);

        robot.s.addLoopAction(() -> {
            robot.getDrivetrain().setWheelPowers(0, 0, 0, 0);
            AprilTagDetection tag = robot.getCamera().getGoalApriltag();
            if(tag != null) {
                isRed = tag.metadata.name == "Red Goal";
            }
        }, 1.3,"Stop");

        robot.s.addImpulseAction(() -> {
            robot.s.killLoopAction("Stop", true);
        },2);

        robot.s.addLoopAction(() -> {
            if(isRed) {robot.getDrivetrain().turnOnPointPID(135);}
            else {robot.getDrivetrain().turnOnPointPID(45);}
            robot.setStatus((byte) 0);
        }, 2,"Turn on Point");

        robot.s.addImpulseAction(() -> {
            robot.s.killLoopAction("Turn on Point", true);
            robot.getDrivetrain().setWheelPowers(0, 0, 0, 0);
        },3.5);

        robot.s.addLoopAction(() -> {
            robot.getDrivetrain().turnOnPointPID(90);
        }, 3.5,"Turn Back");

        robot.s.addImpulseAction(() -> {
            robot.s.killLoopAction("Turn Back", true);
            robot.getDrivetrain().setWheelPowers(0, 0, 0, 0);
        },5);

        // Shoot

        robot.s.addLoopAction(() -> {
            robot.getOuttake().setOuttakeRPM(SHORT_DIST_TOP, SHORT_DIST_BOT);
            robot.getOuttake().runOuttakePID(t);
        }, 3, "Shoot");

        robot.s.addImpulseAction(() -> {
            slot = robot.getIntake().getNextOuttakeSlot();
            robot.getIntake().cycleOuttakeSlot(slot);
        },5);

        robot.s.addImpulseAction(() -> {
            robot.outtake(slot, t);
        },6);

        robot.s.addImpulseAction(() -> {
            slot = robot.getIntake().getNextOuttakeSlot();
            robot.getIntake().cycleOuttakeSlot(slot);
        },9);

        robot.s.addImpulseAction(() -> {
            robot.outtake(slot, t);
        },10);

        robot.s.addImpulseAction(() -> {
            slot = robot.getIntake().getNextOuttakeSlot();
            robot.getIntake().cycleOuttakeSlot(slot);
        },13);

        robot.s.addImpulseAction(() -> {
            robot.outtake(slot, t);
        },14);

        robot.s.addImpulseAction(() -> {
            robot.getIntake().bootkick(BOOT_KICKER_RESTING);
        },15);

        robot.s.addImpulseAction(() -> {
            robot.getOuttake().setOuttakeRPM(0, 0);
        },16);

        robot.s.addLoopAction(() -> {
            if(isRed) {robot.getDrivetrain().turnOnPointPID(45);}
            else {robot.getDrivetrain().turnOnPointPID(135);}
        }, 16, "Turn Away");

        robot.s.addImpulseAction(() -> {
            robot.s.killLoopAction("Turn Away", true);
        },18);

        robot.s.addLoopAction(() -> {
            robot.getDrivetrain().setWheelPowers(1, 1, 1, 1);
        }, 18,"Drive Away");

        robot.s.addImpulseAction(() -> {
            robot.s.killLoopAction("Drive Away", true);
            robot.getDrivetrain().setWheelPowers(0, 0, 0, 0);
        },18.5);

        while (opModeIsActive()) {
            robot.s.loop();

            t.update();
        }
    }
}
