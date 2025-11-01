package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Robot;


@Config
@TeleOp(name = "Angle PID Tuner", group = "Tuning")
public class AnglePIDTuner extends LinearOpMode {

    Robot robot;
    PIDController anglePID;
    private double initHeading = 0;

    public static boolean align = false;
    boolean alignDebounce = false;
    public static double kp = 0, ki = 0, kd = 0;

    public static double goal = 0;
    private double prevGoal = 0;
    public static double error = 0;
    private Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        boolean prevAlign = false;

        anglePID = new PIDController(kp, ki, kd);

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()) {
            if (gamepad1.x && !alignDebounce) {
                align = true;
                alignDebounce = true;
            }

            if(align && !alignDebounce) {
                initHeading = currHeading() % 360;
            }

            if (gamepad1.triangle) {
                align = false;
                alignDebounce = false;
            }

            if(gamepad1.circle) {
                initHeading = currHeading() % 360;
            }

            if(prevAlign != align) {
                anglePID.setPID(kp, ki, kd);
                robot.getDrivetrain().resetImu();
            }

            if(prevGoal != goal) {
                initHeading = currHeading();
                anglePID.setPID(kp, ki, kd);
            }

            fieldCentricDrive();
            prevAlign = align;
            prevGoal = goal;
        }
    }

    public double currHeading() {
        return robot.getDrivetrain().getRobotHeading(AngleUnit.DEGREES);
    }

    private void fieldCentricDrive() {
        double slowdown = gamepad1.right_trigger > 0 ? 0.25 : 1;
        double y = -gamepad1.left_stick_y * slowdown;
        double x = gamepad1.left_stick_x * 1.1 * slowdown;

        double currHeading = currHeading();
        double goalHeading = initHeading*2 - currHeading();
        error = goalHeading - currHeading;
        if(Math.abs(error) > 180) { error = (error + -Math.signum(error) * 180); }
        double alignVal = anglePID.calculate(0, error);
        if(alignVal == 0 || Math.abs(error) <= 2) { align = false; alignDebounce = false;}
        t.addData("Current Heading", currHeading);
        t.addData("Initial Heading", initHeading);
        t.addData("Turn Power", alignVal);
        t.addData("Error", error);
        t.update();

        double rx = align ? alignVal : gamepad1.right_stick_x * slowdown;

        double rotX = x * Math.cos(-currHeading) - y * Math.sin(-currHeading);
        double rotY = x * Math.sin(-currHeading) + y * Math.cos(-currHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frWheelPower = (rotY - rotX - rx) / denominator;
        double flWheelPower = (rotY + rotX + rx) / denominator;
        double brWheelPower = (rotY + rotX - rx) / denominator;
        double blWheelPower = (rotY - rotX + rx) / denominator;

        robot.getDrivetrain().setWheelPowers(flWheelPower, frWheelPower, brWheelPower, blWheelPower);

        if (gamepad1.y) {
            robot.getDrivetrain().resetImu();
        }
    }
}
