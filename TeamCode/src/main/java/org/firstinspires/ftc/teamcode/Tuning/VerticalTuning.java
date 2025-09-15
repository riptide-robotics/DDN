package org.firstinspires.ftc.teamcode.Tuning;

import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_A;
import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_V;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.riptideUtil;


@Config
@TeleOp(name = "Vertical Tuning", group = "Tuning")
public class VerticalTuning extends LinearOpMode {


    Robot robot;
    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    FtcDashboard d = FtcDashboard.getInstance();

    public static double nextGoalXInInches = 0;
    private  double goalXININCHES = 0;

    public static PIDController vertPid = new PIDController(0, 0, 0);
    public double goalInInches = 0;

    public double lastGoalInInches = 0;
    public double startInInches = 0;

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
            goalInInches = goalXININCHES;

            TelemetryPacket f = tuneMovement(new TelemetryPacket());

            f.put("GoalX", goalInInches);
            d.sendTelemetryPacket(f);
            telemetry.addLine("OP mode is active");
            telemetry.update();

        }

    }

    public void updateGoal(){
        if(gamepad1.dpad_up){
            goalXININCHES = nextGoalXInInches;
        }
    }

    public TelemetryPacket tuneMovement(TelemetryPacket p) {
        if (!(lastGoalInInches == goalInInches)) {

            startInInches = lastGoalInInches;
            lastGoalInInches = goalInInches;

            double dx = goalInInches - startInInches;

            motionProfile.setProfile(MAX_A, MAX_V);
            motionProfile.calculateProfile(dx);

            time = System.nanoTime()/1e9;
            elapsedTime = 0;
        }
        else{
            elapsedTime = System.nanoTime()/1e9 - time;
        }

        double currentX = robot.getDrivetrain().getCurrPos().getX(DistanceUnit.INCH);

        p.put("Current X", currentX);

        double linearExpPos = motionProfile.getExpectedPosition(elapsedTime); // where we want to be.

        vertPid.setPID(VERT_KP, riptideUtil.VERT_KI, VERT_KD);
        double xPower = vertPid.calculate(currentX, linearExpPos);

        p.put("power X * 50", xPower * 50);


        robot.getDrivetrain().setWheelPowers(xPower, xPower, xPower, xPower);


        return p;
    }
}
