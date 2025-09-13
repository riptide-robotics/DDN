package org.firstinspires.ftc.teamcode.Modules;

import java.lang.Override;
// Imports to sync
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;

public class Intake {
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

    //
}
