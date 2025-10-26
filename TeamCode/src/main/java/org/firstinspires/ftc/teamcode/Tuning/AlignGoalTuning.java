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

import java.util.ArrayList;

@Config
@TeleOp(name = "Goal Alignment Tuning")
public class AlignGoalTuning extends LinearOpMode {

    /**In degrees.*/
    public static double goal = getDesiredAngle();
    private static double prevGoal = 0;
    Robot robot;

    public static double kp;
    public static double ki;
    public static double kd;


    PIDController controller = new PIDController(kp, ki, kd);

    Telemetry mTele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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



        rotate(mTele,true);
        while (opModeIsActive()) {
            if (prevGoal != goal) {
                prevGoal = goal;
                controller = new PIDController(kp, ki, kd);
                rotate(mTele,true);
            }

            rotate(mTele,false);
            mTele.update();
        }
    }

    double currAngle = 0;
    double prevAngle = 0;
    public void rotate(Telemetry tele, boolean cleanRun) {
        prevAngle = currAngle;
        currAngle = getCurrentAngle();

        if (Math.abs(goal - currAngle) > 180) goal = -(goal - 180);

        double dAngle = prevAngle - currAngle;

        tele.addData("angleChange",dAngle);
        tele.addData("angleGoal",goal);


        double result = controller.calculate(dAngle,goal);


        if (!cleanRun)
            robot.getDrivetrain().setWheelPowers(result,-result,-result,result);




    }

    public double getCurrentAngle() {
        return robot.getDrivetrain().getRobotHeading(AngleUnit.DEGREES);

    }
    public static double getDesiredAngle() {
        throw new UnsupportedOperationException("Waiting on the AprilTags, DO NOT USE THIS!");
    }
}