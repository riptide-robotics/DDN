package org.firstinspires.ftc.teamcode.Autonomous.AutonomousScripts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Drivetrain;

public class Easy3Points extends LinearOpMode {
    Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.startOdometry();

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.setWheelPowers(0.25,0.25,0.25,0.25);
        }
    }
}
