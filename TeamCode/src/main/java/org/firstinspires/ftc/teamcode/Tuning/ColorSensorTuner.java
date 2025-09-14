package org.firstinspires.ftc.teamcode.Tuning;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "Color Sensor Tuner")
public class ColorSensorTuner extends LinearOpMode {
    Robot robot;
    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static float gain;

    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {
        try {
            /* You can give the sensor a gain value, will be multiplied by the sensor's
             * raw value before the normalized color values are calculated. Color sensors
             * (especially the REV Color Sensor V3) can give very low values (depending
             * on the lighting conditions), which only use a small part of the 0-1 range
             * that is available for the red, green, and blue values. In brighter
             * conditions, you should use a smaller gain than in dark conditions. If your
             * gain is too high, all of the colors will report at or near 1, and you won't
             * be able to determine what color you are actually looking at. For this
             * reason, it's better to err on the side of a lower gain (but always greater
             * than  or equal to 1).
             */
            gain = 2;

            /* Once per loop, we will update this hsvValues array. The first element (0)
             * will contain the hue, the second element (1) will contain the saturation,
             * and the third element (2) will contain the value. See
             * http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/
             *     tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color.
             */
            final float[] hsvValues = new float[3];


            // Remove unnecessary idiocy after
            /* xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of
             * the previous and current state of the X button on the gamepad
             */
            boolean xButtonPreviouslyPressed = false;
            boolean xButtonCurrentlyPressed = false;

            /* Get a reference to our sensor object. It's recommended to use
             * NormalizedColorSensor over ColorSensor, because NormalizedColorSensor
             * consistently gives values between 0 and 1, while the values you get
             * from ColorSensor are dependent on the specific sensor you're using.
             */
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

            /* If possible, turn the light on in the beginning (it might already be
             * on anyway, we just make sure it is if we can).
             */
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) colorSensor).enableLight(true);
            }

            /* Wait for the start button to be pressed. */
            waitForStart();

            /* Loop until we are asked to stop */

            while (opModeIsActive()) {
                /* Because someone's memory sucks */
                telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
                telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

                /* Auto gain-edit? I'll deal with that later */
                if (gamepad1.a) {
                    // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                    gain += (float) 0.005;
                } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
                    gain -= (float) 0.005;
                }

                /* idk man maybe use the radius of the contour to know what gain to use? */
                // Show the gain value via telemetry
                telemetry.addData("Gain", gain);

                colorSensor.setGain(gain);
                xButtonCurrentlyPressed = gamepad1.x;
                if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
                    // If the button is (now) down, then toggle the light
                    if (xButtonCurrentlyPressed) {
                        if (colorSensor instanceof SwitchableLight) {
                            SwitchableLight light = (SwitchableLight) colorSensor;
                            light.enableLight(!light.isLightOn());
                        }
                    }
                }
                xButtonPreviouslyPressed = xButtonCurrentlyPressed;

                /* Your favorite RGBA color sensing bc this girl
                 * has no idea how to use HSV hues
                 */
                NormalizedRGBA colors = colorSensor.getNormalizedColors();

                /* Use telemetry to display feedback on the driver station.
                 * We show the red, green, and blue normalized values from
                 * the sensor (in the range of 0 to 1), as well as the
                 * equivalent HSV (hue, saturation and value) values.
                 */

                Color.colorToHSV(colors.toColor(), hsvValues);

                telemetry.addLine()
                        .addData("Red", "%.3f", colors.red)
                        .addData("Green", "%.3f", colors.green)
                        .addData("Blue", "%.3f", colors.blue);
                telemetry.addLine()
                        .addData("Hue", "%.3f", hsvValues[0])
                        .addData("Saturation", "%.3f", hsvValues[1])
                        .addData("Value", "%.3f", hsvValues[2]);
                telemetry.addData("Alpha", "%.3f", colors.alpha);

                /* Just because, if this color sensor also has a distance sensor,
                 * display the measured distance. Note that the reported distance
                 * is only useful at very close range, and is impacted by ambient
                 * light and surface reflectivity.
                 */
                if (colorSensor instanceof DistanceSensor) {
                    telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
                }
                telemetry.update();
            }
        } finally {
            telemetry.addData("End ", "%.3f", ":>");
            telemetry.update();
        }
    }
}
