package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "AAAaaAAAAaaaAA")
//THis class determines how the driving mechanism works, initalizes robot along the way
public class FieldCentricDrive_Owen extends LinearOpMode {

    //The robot to control
    public Robot robot;

    //Prepare the robot for driving
    @Override
    public void runOpMode() throws InterruptedException {

        //prepare the robot to be processed
        robot = new Robot(hardwareMap);

        //send debug message announcing initalization
        telemetry.addData("Add data", "succesfully initiated");
        telemetry.update();

        //make sure that this runs when and only when you ask it to
        waitForStart();
        if (isStopRequested()) return;

        //send debug message announcing that it is starting up
        telemetry.clear();
        telemetry.addData("Robot status", "Started!");
        telemetry.update();

        //Start driving as the user directs
        while (opModeIsActive()) fieldCentricDrive();
    }

    //Determine and send the direction and speed to drive
    public void fieldCentricDrive() {

        //get joysticks
        double slowdown = gamepad1.right_trigger > 0 ? 0.25 : 1;
        double y = -gamepad1.left_stick_y * slowdown;
        double x = gamepad1.left_stick_x * 1.1 * slowdown;
        double rx = gamepad1.right_stick_x * slowdown;

        double heading = robot.getDrivetrain().getRobotHeading(AngleUnit.RADIANS);

        //rotation trig. Determines circle?
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        //math number 2 - this time directed at specific wheels

        double frWheelPower = (rotY - rotX - rx) / denominator;
        double flWheelPower = (rotY + rotX + rx) / denominator;
        double brWheelPower = (rotY + rotX - rx) / denominator;
        double blWheelPower = (rotY - rotX + rx) / denominator;

        //Apply resulting movement to bot
        robot.getDrivetrain().setWheelPowers(flWheelPower, frWheelPower, brWheelPower, blWheelPower);

        //Stop the robot when needed
        if (gamepad1.y) robot.getDrivetrain().resetImu();
    }
}
