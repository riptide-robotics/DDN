package org.firstinspires.ftc.teamcode.Autonomous.AutonomousScripts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tuning.Odometry;

@Config
@Autonomous(name = "Primary Auto")
public class PrimaryAuto extends LinearOpMode {
    //TODO please dont use this yet

 /*
 * Goals for this thing:
 *
 * Start off by driving at an angle (if red 45, if blue -45)
 * Drive for 2*2.82842712 feet (2 squares diagonally)
 * Start launching
 * */


    Drivetrain drivetrain;
    AutonomousRobot robot;
    boolean red = true;
    Outtake outtake;
    Intake intake;
    double time = System.currentTimeMillis();
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap);
        robot = new AutonomousRobot(hardwareMap);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);

        outtake.startFlywheel();
        waitForStart();


        while (opModeIsActive()) {
          //  if ((System.currentTimeMillis() - time)/1e6 < 3)
          //      fieldCentricDrive(0.425, 0.85 * (red ? 1 : -1), Math.toRadians(45) * (red ? 1 : -1));
          //  else {
                intake.spin(1);
                outtake.runOuttakePID(2000, 2100, telemetry);
           // }
        }

    }

    public void fieldCentricDrive(double x, double y, double rx) {

        double heading = robot.getDrivetrain().getRobotHeading(AngleUnit.RADIANS); // heading of bot in radians

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading); // Linear transformations yay
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1); // we like our drivers to have more control
        double frWheelPower = (rotY - rotX - rx) / denominator;
        double flWheelPower = (rotY + rotX + rx) / denominator;
        double brWheelPower = (rotY + rotX - rx) / denominator;
        double blWheelPower = (rotY - rotX + rx) / denominator;

        drivetrain.setWheelPowers(flWheelPower, frWheelPower, brWheelPower, blWheelPower);
    }
}
