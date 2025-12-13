package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.riptideUtil.TEAM_COLOR;

@Config
@TeleOp(name = "Turntable + Camera")
public class TurnTableAndCameraTest extends LinearOpMode {
    Robot robot;
    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    public static double kp = 0.002;
    public static double ki = 0.003;
    public static double kd = 0.00005;
    public static double kf = 0;
    public static double deadZone = 5; // Never tested these five

    double currPosDeg = 0;
    double currPosTicks = 0;

    public static double goalDeg = 0;
    double prevGoalDeg;
    double goalTicks = 0;
    static final double ticksToDegrees = 360/751.8;
    static final double degreesToTicks = 751.8/360;

    public ElapsedTime startTime= new ElapsedTime();

    PIDController motorController;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.setTeamColor(TEAM_COLOR.RED);
        Double error;
        Double absoluteError;
        Double dist;
        Double absoluteGoal;

        motorController = new PIDController(kp, ki, kd);

        waitForStart();

        while (opModeIsActive()) {
            error = robot.getCamera().getGoalAngleError();
            t.addData("Calculated error", error);
            if (error == null) {
                t.addData("Absolute error", "Null");
                absoluteError = null;
                absoluteGoal = null;
            } else {
                absoluteError = robot.getCamera().getAbsoluteAngleError(robot.getDrivetrain().getRobotHeading(AngleUnit.DEGREES), error);
                absoluteGoal = robot.getCamera().getAbsoluteAngleGoal(robot.getDrivetrain().getRobotHeading(AngleUnit.DEGREES), error);
                t.addData("Absolute error", absoluteError);
            }
            int len = robot.getCamera().getTagDetections().size();
            String s = "";
            for (int i = 0; i < len; i++) {
                s = s + robot.getCamera().getTagDetections().get(0) + ", ";
            }
            t.addData("Tag(s) detected", s);
            dist = robot.getCamera().getGoalDistance();
            t.addData("Distance", dist);
            prevGoalDeg = 0;

            setTurnPID(hardwareMap.dcMotor.get("turnTable"), absoluteGoal != null ? absoluteGoal : prevGoalDeg);
            t.update();
        }
    }

    public void setTurnPID(DcMotor motor, Double goalDeg) {
        if (goalDeg == null) {
            goalDeg = prevGoalDeg;
        }
        if(prevGoalDeg != goalDeg) {
            startTime.reset();
            prevGoalDeg = goalDeg;
        }
        motorController.setPID(kp, ki, kd);

        double goalTicks = goalDeg * degreesToTicks;

        currPosTicks = motor.getCurrentPosition();
        double setPower = motorController.calculate(currPosTicks, goalTicks) + kf;
        motor.setPower(setPower);
        currPosDeg = currPosTicks * ticksToDegrees;

        t.addData("Current Position", currPosDeg);
        t.addData("Goal Position", goalDeg);
        t.addData("Set Power", setPower);
    }
}
