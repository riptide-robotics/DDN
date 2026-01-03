package org.firstinspires.ftc.teamcode.UnitTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Outtake;

@TeleOp(name = "Sequencer Turntable Test")
public class TurntableTest extends LinearOpMode {
    Outtake outtake;
    double goalAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            turntable();
        }
    }
    public void turntable() {
        if (gamepad1.x) {
//            outtake.SetTurretGoalAngle(Math.min(1,goalAngle += 0.5));
//            outtake.SetTurretGoalAngle(Math.max(-1,goalAngle -= 0.5));
        }
    }
}
