package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.riptideUtil;

// If we're using diffy then this would have to be rewritten a bit but otherwise usefull
// ----- READY TO TRANSFER ----- //
@Config
@TeleOp(name = "Angle PID Tuner")
public class AnglePIDTuner extends LinearOpMode {
    public static double goalAngle = 0;
    private double prevGoalAngle = goalAngle;
    private static Pose2D goal = new Pose2D(DistanceUnit.INCH, 1000 * Math.cos(Math.toRadians(goalAngle)), 1000 * Math.sin(Math.toRadians(goalAngle)), AngleUnit.DEGREES, 0);

    Robot robot;

    public static double kp = 0.0;
    public static double ki = 0.0;
    public static double kd = 0.0;
    private double prevkp = kp;
    private double prevki = ki;
    private double prevkd = kd;

    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Robot(hardwareMap);

        robot.getDrivetrain().startOdometry();
        robot.getDrivetrain().getPinpoint().setPosition(riptideUtil.START_POSITION);

        robot.getDrivetrain().setTurnController(kp, ki, kd);

        telemetry.addData("Robot status", "successfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            if(gamepad1.a) {robot.getDrivetrain().getPinpoint().setHeading(0, AngleUnit.DEGREES);}

            if(prevGoalAngle != goalAngle) {
                goal = new Pose2D(DistanceUnit.INCH, 1000*Math.cos(Math.toRadians(goalAngle)), 1000*Math.sin(Math.toRadians(goalAngle)), AngleUnit.DEGREES, 0);
                prevGoalAngle = goalAngle;
            }
            if (prevkp != kp || prevki != ki || prevkd != kd) {
                robot.getDrivetrain().setTurnController(kp, ki, kd);
                prevkp = kp;
                prevki = ki;
                prevkd = kd;
            }
            robot.getDrivetrain().goToPosPID(goal);

            telem();
        }
    }


    public void telem() {
        t.addData("Robot Heading", robot.getDrivetrain().getCurrPos().getHeading(AngleUnit.DEGREES));
        t.addData("Robot Heading 2nd Value", robot.getDrivetrain().getRobotHeading(AngleUnit.DEGREES));
        t.addData("Angle Difference", robot.getDrivetrain().getAngleDiff());
        t.addData("Angle PID Value", robot.getDrivetrain().getAnglePower());
        t.addData("Angle atan2(dy, dx)", robot.getDrivetrain().getAngleATan());
        t.addData("Goal Angle: ", goalAngle);
        t.addData("Goal X: ", goal.getX(DistanceUnit.INCH));
        t.addData("Goal Y: ", goal.getY(DistanceUnit.INCH));
        t.addData("Bot X: ", robot.getDrivetrain().getPinpoint().getPosX(DistanceUnit.INCH));
        t.addData("Bot Y: ", robot.getDrivetrain().getPinpoint().getPosY(DistanceUnit.INCH));
        t.addData("Left Wheel Powers", robot.getDrivetrain().getWheelPowersArray()[0]);
        t.addData("Right Wheel Powers", robot.getDrivetrain().getWheelPowersArray()[1]);
        t.update();
    }
}
