package org.firstinspires.ftc.teamcode.Tuning;

import static org.firstinspires.ftc.teamcode.riptideUtil.*;

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

@Config
@TeleOp(name = "PID to Point Tuner")
public class PIDToPointTuner extends LinearOpMode {
    public static double x = 0;
    public static double y = 0;
    public static boolean goToPoint = false;
    private double prevx = x;
    private double prevy = y;
    private static Pose2D goal = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, 0);


    Robot robot;

    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        robot.getDrivetrain().getPinpoint().resetPosAndIMU();
        robot.getDrivetrain().startOdometry();
        //robot.getDrivetrain().setPinpointPos(riptideUtil.START_POSITION);

        telemetry.addData("Robot status", "successfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            //if(gamepad1.a) {robot.getDrivetrain().resetCurrPos();}
            if (gamepad1.a) {
                robot.getDrivetrain().getPinpoint().setPosition(START_POSITION);
                t.addLine("ResetPosition");
            }

            if (prevx != x || prevy != y) {
                goal = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, 0);
                prevx = x;
                prevy = y;
            }

            if (goToPoint) {
                robot.getDrivetrain().goToPosPID(goal);
            } else {
                robot.getDrivetrain().setWheelPowers(0, 0, 0, 0);
            }

            telem();
        }
    }

    public void telem() {
        t.addData("Robot X", robot.getDrivetrain().getCurrPos().getX(DistanceUnit.INCH));
        t.addData("Robot Y", robot.getDrivetrain().getCurrPos().getY(DistanceUnit.INCH));
        t.addData("Robot Heading", robot.getDrivetrain().getCurrPos().getHeading(AngleUnit.DEGREES));
        t.addData("Goal X", x);
        t.addData("Goal Y", y);
        t.addData("Distance", robot.getDrivetrain().getFbDist());
        t.addData("FB Error", robot.getDrivetrain().getFbPower());
        t.addData("FB Dot Product", robot.getDrivetrain().getFbVectNorm());
        t.addData("Angle Difference", robot.getDrivetrain().getAngleDiff());
        t.addData("Angle atan2(dy, dx)", robot.getDrivetrain().getAngleATan());
        t.addData("Left Wheel Powers", robot.getDrivetrain().getWheelPowersArray()[0]);
        t.addData("Right Wheel Powers", robot.getDrivetrain().getWheelPowersArray()[1]);
        t.update();
    }
}
