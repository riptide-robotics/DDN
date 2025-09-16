package org.firstinspires.ftc.teamcode.Autonomous.AutonomousScripts;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Path;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Waypoint;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Utils.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Locale;

@Config
@Autonomous(name = "TEST AUTO")
public class TestAuto extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime();
    AutonomousRobot autoRobot;
    Drivetrain drivetrain;


    double oldTime = 0;


    Path p = new Path.PathBuilder()
            .addNewFullPoint(
                    new Waypoint(1, 72, 90, 50, 25, DistanceUnit.INCH),
                    () -> {},
                    1)

            .addNewFullPoint(
                    new Waypoint(24, 72, 90, 50, 25, DistanceUnit.INCH),
                    () -> {},
                    1)

            .build();





    @Override
    public void runOpMode() throws InterruptedException {
        autoRobot = new AutonomousRobot(hardwareMap);

        autoRobot.setPath(p);
        autoRobot.startPath();
        drivetrain.startOdometry(true);


        waitForStart();
        timer.reset();
        resetRuntime();

        while(opModeIsActive()){
            updatePinPoint();
            autoRobot.step();
        }
    }

    public void updatePinPoint(){

        autoRobot.getOdoComputer().update();

        // Calculates time per each cycle and finds number of updates per seconds (frequency)
        double newTime = getRuntime();
        oldTime = newTime;

        Pose2D pos = autoRobot.getOdoComputer().getPosition();

        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", autoRobot.getOdoComputer().getVelX(DistanceUnit.MM), autoRobot.getOdoComputer().getVelY(DistanceUnit.MM), autoRobot.getOdoComputer().getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);

        telemetry.addData("Status", autoRobot.getOdoComputer().getDeviceStatus());

        telemetry.addData("Pinpoint Frequency", autoRobot.getOdoComputer().getFrequency()); //prints/gets the current refresh rate of the Pinpoint

        telemetry.addData("Current path point", autoRobot.getPathIndex());
        telemetry.addData("At Point", autoRobot.isAtPoint());
        telemetry.addData("Elapsed Time", autoRobot.getElapsedTime());
        telemetry.addData("Delay", autoRobot.getDelay());
        telemetry.addData("Auton Real Position", autoRobot.autonrealPos());
        telemetry.addData("Auton Pos", autoRobot.autonPos());
        telemetry.addData("Robot not at end of path ", autoRobot.isNotAtEndOfPath());
        //telemetry.addData("Past delay to next point", autoRobot.pastDelayUntilNextPoint());
        telemetry.addData("Path size", autoRobot.pathsize());
        telemetry.update();
    }

}
