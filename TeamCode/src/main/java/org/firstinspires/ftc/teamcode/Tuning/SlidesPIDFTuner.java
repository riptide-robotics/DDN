package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "Vertical slides PIDF Tuner", group = "Tuning")
public class SlidesPIDFTuner extends LinearOpMode {

    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    Robot robot;

    public static double kp = 0.02;     // idfk
    public static double ki = 0.3;      // dude
    public static double kd = 0.001;    // eff this sh1t
    public static double kf = 0;        // /^\  T^T ToT T-T T_T TOT

    double currentPosition;
    boolean TorF = false;

    public static double goal = 0;

    double prevGoal;

    public ElapsedTime startTime= new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        PIDController slideController = new PIDController(kp, ki, kd);

        waitForStart();

        while(opModeIsActive()){

            if(prevGoal != goal) {
                startTime.reset();
                prevGoal = goal;
            }

            slideController.setPID(kp, ki, kd);
//            currentPosition = robot.getSlides().getSlidePosition();

            //robot.getSlides().setSlidePower(slideController.calculate(currentPosition, goal) + kf);

            t.addData("Current Position", currentPosition);
            t.addData("Goal Position", goal);
            t.update();

        }
    }
}
