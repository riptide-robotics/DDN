package org.firstinspires.ftc.teamcode.Autonomous.AutonomousScripts;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Drivetrain;

@Autonomous(name="Meet0")
public class Easy3Points extends LinearOpMode {
    Drivetrain drivetrain;


    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new Drivetrain(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.setWheelPowers(1,1,1,1);
            Thread.sleep(3000);
            drivetrain.setWheelPowers(0,0,0,0);
            break;
        }
    }
}
