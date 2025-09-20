package org.firstinspires.ftc.teamcode.Tuning;

import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_A;
import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_V;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.riptideUtil;

public class VerticalTuning extends LinearOpMode {

    Robot robot;
    FtcDashboard d = FtcDashboard.getInstance();
    public static double nextGoalXInInches = 0;
    private double goalXININCHES = 0;
    public static PIDController vertPid = new PIDController(0, 0, 0);
    private double lastGoal = 0;
    private double startPos = 0;
    private double elapsedTime;
    private double time = System.nanoTime() / (Math.pow(10, 9));

    private TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(0, 0);

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
        robot.getDrivetrain().startOdometry();
        /*
         * * * * * * * * * * * * * * *
         * LOOP
         * * * * * * * * * * * * * * *
         */
        while (opModeIsActive()) {
            updateGoal();
            TelemetryPacket f = tuneMovement(new TelemetryPacket());

            f.put("GoalX", goalXININCHES);
            d.sendTelemetryPacket(f);
            telemetry.addLine("OP mode is active");
            telemetry.update();

        }
    }

    private void updateGoal() {
        if (gamepad1.dpad_up)
            goalXININCHES = nextGoalXInInches;

    }

    private TelemetryPacket tuneMovement(TelemetryPacket p) {
        if (lastGoal != goalXININCHES) {
            startPos = lastGoal;
            lastGoal = goalXININCHES;

            double dx = goalXININCHES - lastGoal;

            motionProfile.setProfile(MAX_A, MAX_V);
            motionProfile.calculateProfile(dx);

            time = System.nanoTime() / 1e9;
            elapsedTime = 0;
        }
        else
            elapsedTime = System.nanoTime()/1e9 - time;

        double currentX = robot.getDrivetrain().getCurrPos().getX(DistanceUnit.INCH);

        p.put("Current X", currentX);

        double expX = startPos * motionProfile.getExpectedPosition(elapsedTime);
        p.put("exp x", expX);
        vertPid.setPID(VERT_KP, riptideUtil.VERT_KI, VERT_KD);

        double xPower = vertPid.calculate(currentX, expX);

        robot.getDrivetrain().setWheelPowers(xPower, xPower, xPower, xPower);

        return p;
    }
}