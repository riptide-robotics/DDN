package org.firstinspires.ftc.teamcode.UnitTests;

import static org.firstinspires.ftc.teamcode.riptideUtil.angularVelocity;
import static org.firstinspires.ftc.teamcode.riptideUtil.econserved;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Camera;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "Power on Distance")
public class PowerOnDistance extends LinearOpMode {
    Robot robot;
    Telemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap);

        Camera.processors_enabled processor = Camera.processors_enabled.ALL;
        robot.getCamera().setPipeline(processor);


        waitForStart();
        while(opModeIsActive()){
            robot.setFlyWheelPowerOnDistance(true, tele);
            tele.update();

            for (AprilTagDetection d : robot.getCamera().getTagDetections()) {
                telemetry.addLine("Detected tag name: " + d.metadata.name + " id: " + d.id);
            }
            telemetry.update();
        }
    }
}
