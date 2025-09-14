package org.firstinspires.ftc.teamcode.UnitTests;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.revrobotics.ColorSensorV3;

public class RGBSensor extends LinearOpMode {
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
        ColorSensorV3.RawColor detectedColor = colorSensor.getRawColor();

        int red = rawColor.red;
        int green = rawColor.green;
        int blue = rawColor.blue;
        int alpha = rawColor.alpha;

        telemetry.addData("Raw Red: ", red);
        telemetry.addData("Raw Green: ", green);
        telemetry.addData("Raw Blue: ", blue);
        telemetry.update();
        //

    }
}
