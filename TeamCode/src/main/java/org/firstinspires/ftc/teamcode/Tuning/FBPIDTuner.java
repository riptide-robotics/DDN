package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.riptideUtil;

@Config
@TeleOp(name="Forward Backward PID Tuner")
public class FBPIDTuner extends LinearOpMode {
    // this should be 10 inches parallel to the robot
    // tune for 40 inches
    public static double x = 0;
    public static double y = 0;
    private double prevx = x;
    private double prevy = y;
    private static Pose2D goal = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, 0);

    Robot robot;

    public static double kp = 0.0365;
    public static double ki = 0.0075;
    public static double kd = 0.0005;
    private double prevkp = kp;
    private double prevki = ki;
    private double prevkd = kd;

    //PIDController controller = new PIDController(kp, ki, kd);

    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        robot.getDrivetrain().startOdometry();
        robot.getDrivetrain().getPinpoint().setPosition(riptideUtil.START_POSITION);

        robot.getDrivetrain().setForwardController(kp, ki, kd);

        telemetry.addData("Robot status", "successfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()) {
            if(gamepad1.a) {robot.getDrivetrain().resetCurrPos();}

            robot.getDrivetrain().getPinpoint().update();
            robot.getDrivetrain().getPinpoint().getCurrPos();
            if(prevx != x || prevy != y) {
                goal = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, 0);
                prevx = x;
                prevy = y;
            }
            if (prevkp != kp || prevki != ki || prevkd != kd) {
                robot.getDrivetrain().setForwardController(kp, ki, kd);
                prevkp = kp;
                prevki = ki;
                prevkd = kd;
            }
            robot.getDrivetrain().goToPosPID(goal);

            telem();
        }
    }

    public void telem() {
        t.addData("Robot X", robot.getDrivetrain().getCurrPos().getX(DistanceUnit.INCH));
        t.addData("Robot Y", robot.getDrivetrain().getCurrPos().getY(DistanceUnit.INCH));
        t.addData("Goal X: ", x);
        t.addData("Goal Y: ", y);
        t.addData("Robot Heading", robot.getDrivetrain().getRobotHeading(AngleUnit.DEGREES));
        t.addData("Left Wheel Powers", robot.getDrivetrain().getWheelPowersArray()[0]);
        t.addData("Right Wheel Powers", robot.getDrivetrain().getWheelPowersArray()[1]);
        t.addData("FB Error", robot.getDrivetrain().getFbPower());
        t.addData("FB Distance", robot.getDrivetrain().getFbDist());
        t.addData("FB Dot Product", robot.getDrivetrain().getFbVectNorm());
        t.update();
    }
}
