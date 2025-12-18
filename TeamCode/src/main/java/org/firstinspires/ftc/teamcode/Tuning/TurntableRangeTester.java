package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Turntable Range Tester", group = "Tuning")
public class TurntableRangeTester extends LinearOpMode {

    Robot robot;

    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            robot.getOuttake().mapJoyToAngle(gamepad1.left_stick_x);

            robot.getOuttake().updateTurntableAngle(t);

            t.update();
        }

    }
}
