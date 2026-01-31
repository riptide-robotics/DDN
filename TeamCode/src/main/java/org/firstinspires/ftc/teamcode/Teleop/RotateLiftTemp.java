package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.EndgameServos;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "RotateLiftServos")

public class RotateLiftTemp extends LinearOpMode {
    Robot robot;
    EndgameServos servos;
    @Override
    public void runOpMode() throws InterruptedException {

        servos = new EndgameServos(hardwareMap);

        robot = new Robot(hardwareMap);
        telemetry.addData("Robot status", "succesfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clear();
        telemetry.addData("Robot status", "Started!");
        telemetry.update();

        while (opModeIsActive()) {
            servos.lift(0);
        }
    }

}
