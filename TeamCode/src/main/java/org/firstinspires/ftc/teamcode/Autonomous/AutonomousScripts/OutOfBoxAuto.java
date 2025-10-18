package org.firstinspires.ftc.teamcode.Autonomous.AutonomousScripts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@Autonomous(name = "Meet 0")
public class OutOfBoxAuto extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException{

        double speed = 1;
        boolean hasRun = false;

        robot = new Robot(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            if (!hasRun) {
                robot.getDrivetrain().setWheelPowers(speed, speed, speed, speed);
                sleep(3);
                robot.getDrivetrain().setWheelPowers(0,0,0,0);
            }
        }
    }
}
