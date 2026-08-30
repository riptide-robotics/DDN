package org.firstinspires.ftc.teamcode.Autonomous.AutonomousScripts;

// it would be very sad if we would actually have to transfer this so...

// DO NOT TRANSFER

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot;
@Config
@Autonomous(name="DRIVE FORWARD")
public class Easy3Points extends LinearOpMode {
    Drivetrain drivetrain;
    Robot robot;
    boolean run = false;
    ElapsedTime timer;
    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        drivetrain = new Drivetrain(hardwareMap);
        robot = new Robot(hardwareMap);

        waitForStart();
        timer.reset();
        timer.startTime();

        while (opModeIsActive()) {
                if (!run) {
                    robot.getDrivetrain().setWheelPowers(
                            -1,
                            -1,
                            -1,
                            -1
                    );
                    if (timer.milliseconds() > 500) {
                        run = true;
                    }
            }
        }
    }
}
