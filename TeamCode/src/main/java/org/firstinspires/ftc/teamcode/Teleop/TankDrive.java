package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp(name = "tankDrive")
public class TankDrive extends LinearOpMode {

    public Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);
        telemetry.addData("Robot status", "succesfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clear();
        telemetry.addData("Robot status", "Started!");
        telemetry.update();

        while (opModeIsActive()) {
            tankDrive();
        }
    }
    public void tankDrive() {
        robot.getDrivetrain().setWheelPowers(
                -gamepad1.left_stick_y,
                -gamepad1.right_stick_y,
                -gamepad1.right_stick_y,
                -gamepad1.left_stick_y
                );
    }
}
