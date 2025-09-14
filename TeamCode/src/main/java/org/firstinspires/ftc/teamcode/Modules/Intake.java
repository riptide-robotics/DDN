package org.firstinspires.ftc.teamcode.Modules;

import java.lang.Override;
// Imports to sync
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;

public class Intake extends LinearOpMode {
    /**
     * Change the I2C port below to match the connection of your color sensor
     */
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a
     * parameter. The device will be automatically initialized with default
     * parameters.
     */
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    /**
     * A Rev Color Match object is used to register and detect known colors. This can
     * be calibrated ahead of time or during operation.
     *
     * This object uses a simple euclidian distance to estimate the closest match
     * with given confidence range.
     */
    private final ColorMatch colorMatcher = new ColorMatch();

    /**
     * These colors must be recalibrated to represent purple & green
     */
    private final Color purpleTarget = new Color(0.343, 0.127, 0.429);
    private final Color greenTarget = new Color(0.197, 0.561, 0.240);

    public void robotInit() {
        colorMatcher.addColorMatch(purpleTarget);
        colorMatcher.addColorMatch(greenTarget);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * The method GetColor() returns a normalized color value from the sensor and can be
         * useful if outputting the color to an RGB LED or similar. To
         * read the raw color, use GetRawColor().
         *
         * The color sensor works best when within a few inches from an object in
         * well lit conditions (the built in LED is a big help here!). The farther
         * an object is the more light from the surroundings will bleed into the
         * measurements and make it difficult to accurately determine its color.
         */
        Color detectedColor = colorSensor.getColor();

        /**
         * Run the color match algorithm on our detected color
         */
        String colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    }
}
