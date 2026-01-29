package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.Camera;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.riptideUtil.TEAM_COLOR;

@TeleOp(name = "Camera Tester")
public class CameraTester extends LinearOpMode {
    Robot robot;
    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    Double error;
    double absoluteError;
    Double dist;
    Outtake outtake;
    Camera camera;
    Drivetrain drivetrain;

    public void runOpMode() {
        outtake = new Outtake(hardwareMap);
        camera = new Camera(hardwareMap, TEAM_COLOR.RED);
        drivetrain = new Drivetrain(hardwareMap);
        //robot = new Robot(hardwareMap);
        //robot.setTeamColor(TEAM_COLOR.RED);

        waitForStart();

        while (opModeIsActive()) {
            error = camera.getGoalAngleError();
            t.addData("Calculated error", error);
            if (error == null) {
                t.addData("Absolute error", "Null");
            } else {
                absoluteError = camera.getAbsoluteAngleError(drivetrain.getRobotHeading(AngleUnit.DEGREES), error);
                t.addData("Absolute error", absoluteError);
            }
            int len = camera.getTagDetections().size();
            String s = "";
            for (int i = 0; i < len; i++) {
                s = s + camera.getTagDetections().get(0) + ", ";
            }
            t.addData("Tag(s) detected", s);
            dist = camera.getGoalDistance();
            t.addData("Distance", dist);
            char[] motifOrder = camera.scanMotifOrder();
            t.addData("Scanned Motif Order", new String(motifOrder));
            t.update();
        }
    }
}
