package org.firstinspires.ftc.teamcode.UnitTests;

// this can be tranfered but only for referance, I don't think we're actually going to use this

// READY TO TRANSFER

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "Color Sensor Test")
public class ColorSensorTest extends LinearOpMode {
    Robot robot;
    String[] order = new String[3];
    public static float gain = 67;
    int[] pgratio = new int[2];

    NormalizedColorSensor colorSensor;

    Intake intake;

    public void runOpMode() {
        char sendColor;
        char prevString = 'a';

        robot = new Robot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            robot.getIntake().setGain(gain);
            sendColor = robot.getIntake().checkColor();
            if (sendColor != prevString) {
                prevString = sendColor;
                telemetry.addData("Color Detected", sendColor);
            }
            telemetry.addData("Current color ", sendColor);
            telemetry.update();
        }
    }
}
