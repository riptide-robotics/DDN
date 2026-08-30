package org.firstinspires.ftc.teamcode.UnitTests;

// DO NOT TRANSFER

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.TurnTable;

@Config
@TeleOp(name = "Sequencer Turntable Test")
public class TurntableTest extends LinearOpMode {
    double prevGoalAngle = 0;
    public static double goalAngle = 0;
    TurnTable turnTable;
    public static double kp, ki, kd = 0;
    // kp = 0.01, ki = 0.01, kd = 0.0002

    boolean dpadUpWasPressed,
            dpadDownWasPressed,
            xWasPressed,
            yWasPressed,
            aWasPressed,
            bWasPressed
                    = false;

    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        turnTable = new TurnTable(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            t.addData("Current position", (turnTable.getAngle() / 3));
            t.addData("Current goal", goalAngle);
            t.addData("Error", (goalAngle - (turnTable.getAngle() / 3)));

            if (gamepad1.dpad_up && !dpadUpWasPressed) {
                kp += 0.01;
                dpadUpWasPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpWasPressed = false;
            }
            if (gamepad1.dpad_down && !dpadDownWasPressed) {
                kp -= 0.01;
                dpadDownWasPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownWasPressed = false;
            }
            if (gamepad1.y && !yWasPressed) {
                ki += 0.01;
                yWasPressed = true;
            } else if (!gamepad1.y) {
                yWasPressed = false;
            }
            if (gamepad1.a && !aWasPressed) {
                ki -= 0.01;
                aWasPressed = true;
            } else if (!gamepad1.a) {
                aWasPressed = false;
            }
            if (gamepad1.x && !xWasPressed) {
                kd += 0.002;
                xWasPressed = true;
            } else if (!gamepad1.x) {
                xWasPressed = false;
            }
            if (gamepad1.b && !bWasPressed) {
                kd -= 0.002;
                bWasPressed = true;
            } else if (!gamepad1.b) {
                bWasPressed = false;
            }

            if (prevGoalAngle != goalAngle) {
                turnTable.getPIDController().setPID(kp, ki, kd);
                turnTable.getPIDController().resetIntegral();
                turnTable.setGoalAngle(goalAngle);
                prevGoalAngle = goalAngle;
            }
            turnTable.goToGoalAngle();

            t.update();
        }
    }
}
