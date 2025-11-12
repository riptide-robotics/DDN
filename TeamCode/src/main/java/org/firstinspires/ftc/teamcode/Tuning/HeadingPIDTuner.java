package org.firstinspires.ftc.teamcode.Tuning;

import static java.lang.Math.abs;

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
@TeleOp(name = "Goal Alignment Tuning")
public class HeadingPIDTuner extends LinearOpMode {

    /**In degrees.*/
    public static double goal = 0 /*TODO get desired angle*/;
    private static double prevGoal = 0;

    private double initHeading = 0;
    Robot robot;

    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;

    PIDController controller = new PIDController(kp, ki, kd);

    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * * * * * * * * * * * * * * *
         * INITIALIZATION
         * * * * * * * * * * * * * * *
         */
        robot = new Robot(hardwareMap);

        telemetry.addData("Robot status", "successfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // * * * * * * * * * * * * * * *
        // * Start button clicked
        // * * * * * * * * * * * * * * *

        telemetry.clear();

        while (opModeIsActive()) {
            if (prevGoal != goal) {
                prevGoal = goal;
                controller = new PIDController(kp, ki, kd);
                initHeading = getCurrentAngle();
            }

            double rot = getTurnValue();
            robot.getDrivetrain().setWheelPowers(-rot,rot,rot,-rot);
            t.update();
        }
    }

    public double getTurnValue() {
        double currAngle = getCurrentAngle();

        double error =prevGoal - currAngle;

        if (Math.abs(error) > 180) {error += Math.signum(error) * 360;}

        t.addData("Goal ", goal);
        t.addData("Error ", error);
        t.addData("Current Heading ", currAngle);

        return controller.calculate(0, error);
    }


    public double getCurrentAngle() {
        return robot.getDrivetrain().getRobotHeading(AngleUnit.DEGREES);

    }
}