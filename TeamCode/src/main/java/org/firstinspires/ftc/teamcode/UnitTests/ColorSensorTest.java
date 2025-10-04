package org.firstinspires.ftc.teamcode.UnitTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Robot;

public class ColorSensorTest extends LinearOpMode {
    String[] order = new String[3];
    int[] pgratio = new int[2];

    NormalizedColorSensor colorSensor;
    float gain = 2; // ...... ANYWAY

    Intake intake;

    public void runOpMode() {
        String sendColor;
        String prevString = "";

        intake = new Intake(hardwareMap);

        while (opModeIsActive()) {
            sendColor = intake.scanColor();
            if (!sendColor.equals(prevString)) {
                telemetry.addData("Color Detected", sendColor);
            }
            telemetry.update();
        }
    }
}
