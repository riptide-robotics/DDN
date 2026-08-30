package org.firstinspires.ftc.teamcode.UnitTests;

// DO NOT TRANSFER

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Camera;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.riptideUtil.TEAM_COLOR;

@Config
@TeleOp(name = "OuttakediCamera")
public class OuttakediCamera extends LinearOpMode {
    Camera camera;
    Outtake outtake;

    public static double shortlaunchlow = 0;
    public static double shortlaunchhigh = 5;
    public static double farlaunchlow = 20;
    public static double farlaunchhigh = 30;
    public static double nolaunchlow = 7;
    public static double nolaunchhigh = 14;

    public static double lowLaunchSpeed = 50;
    public static double highLaunchSpeed = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new Camera(hardwareMap, TEAM_COLOR.RED);
        outtake = new Outtake(hardwareMap);

        waitForStart();
        Double distance;

        while (opModeIsActive()) {
            distance = camera.getDistanceToRedGoal();
            telemetry.addData("Distance calculated", distance);
            if (distance == null) {
                telemetry.addLine("Range: null");
                continue;
            }
            if (distance >= shortlaunchlow && distance <= shortlaunchhigh) {
                telemetry.addLine("Range: low");
                telemetry.addLine();
                outtake.setPowerOnDist(lowLaunchSpeed, true, telemetry);
            } else if (distance >= farlaunchlow && distance <= farlaunchhigh) {
                telemetry.addLine("Range: mid");
                telemetry.addLine();
                outtake.setPowerOnDist(highLaunchSpeed, true, telemetry);
            } else if (distance >= nolaunchlow && distance <= nolaunchhigh) {
                telemetry.addLine("Range: high");
                telemetry.addLine();
                outtake.setPowerOnDist(0.0, false, telemetry);
            }
            telemetry.update();
        }
    }
}
