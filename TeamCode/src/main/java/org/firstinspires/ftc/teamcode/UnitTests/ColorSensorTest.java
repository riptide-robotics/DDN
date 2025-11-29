package org.firstinspires.ftc.teamcode.UnitTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Color Sensor Test")
public class ColorSensorTest extends LinearOpMode {
    Robot robot;
    String[] order = new String[3];
    int[] pgratio = new int[2];

    NormalizedColorSensor colorSensor;
    float gain = 2; // ...... ANYWAY

    Intake intake;

    public void runOpMode() {
        char sendColor;
        char prevString = 'a';

        robot = new Robot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            sendColor = robot.getIntake().scanColor();
            if (!(sendColor == prevString)) {
                prevString = sendColor;
                telemetry.addData("Color Detected", sendColor);
            }
            telemetry.update();
        }
    }
}
