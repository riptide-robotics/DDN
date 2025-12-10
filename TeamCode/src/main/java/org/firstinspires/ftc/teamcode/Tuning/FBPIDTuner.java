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
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name="Forward Backward PID Tuner")
public class FBPIDTuner extends LinearOpMode {
    // this should be 10 inches perpendicular to the robot
    public static double x = 10;
    public static double y = 0;
    public static Pose2D goal = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, 0);

    Robot robot;

    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;

    PIDController controller = new PIDController(kp, ki, kd);

    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        telemetry.addData("Robot status", "successfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()) {
            pidVal = getFBVal();
        }
    }

    public double getFBVal() {
        double dx = goal.getX(DistanceUnit.INCH) - pinpoint.getPosX(DistanceUnit.INCH);
        double dy = goal.getY(DistanceUnit.INCH) - pinpoint.getPosY(DistanceUnit.INCH);
        double distanceToPoint = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

        double headingx = Math.cos(getRobotHeading(AngleUnit.RADIANS));
        double headingy = Math.sin(getRobotHeading(AngleUnit.RADIANS));
        double[] headingVect = {headingx, headingy};
        double[] pointVect = {dx/distanceToPoint, dy/distanceToPoint};
        double fbError = headingVect[0] * pointVect[0] + headingVect[1] * pointVect[1];

        return controller.calculate(0, fbError);
    }
}
