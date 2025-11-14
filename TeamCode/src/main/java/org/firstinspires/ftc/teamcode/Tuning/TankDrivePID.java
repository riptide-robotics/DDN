package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name="tankDrivePIDTuning", group="tuning")
public class TankDrivePID extends LinearOpMode {
    Robot robot;

    public static double VerticalDistanceInInches = 0;
    public double goal = 0;
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public final PIDController vertController = new PIDController(kp, ki, kd);

    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * * * * * * * * * * * * * * *
         * INITIALIZATION
         * * * * * * * * * * * * * * *
         */
        robot = new Robot(hardwareMap);

        t.addData("Robot status", "successfully initiated");
        t.update();

        waitForStart();
        if (isStopRequested()) return;

        // * * * * * * * * * * * * * * *
        // * Start button clicked
        // * * * * * * * * * * * * * * *

        t.clear();

        while (opModeIsActive()) {
            if (goal != VerticalDistanceInInches) {
                goal = VerticalDistanceInInches;
                vertController.setPID(kp, ki, kd);
            }

            //double power = setWheelPower(t);

            t.update();


        }


    }
}
