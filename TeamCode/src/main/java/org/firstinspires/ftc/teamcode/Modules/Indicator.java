package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.Placeholder;


@Placeholder(note = "Waiting on an RGB light")
public class Indicator {
    private final HardwareMap hardwareMap;
    private final I2cDeviceSynch rgbLight;
    public Indicator(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.rgbLight = hardwareMap.i2cDeviceSynch.get("Indicator");
    }

    public enum statusLights {
        EMPTY((byte) 0,(byte) 0,(byte) 0),
        SEMI_OPEN((byte) 0,(byte) 0,(byte) 0),
        SEMI_OPEN_AND_NONE_REQUESTED((byte) 0,(byte) 0,(byte) 0),
        FULL_SPINDEXER((byte) 0,(byte) 0,(byte) 0),
        FULL_SPINDEXER_AND_NONE_REQUESTED((byte) 0,(byte) 0,(byte) 0);

        final byte r;
        final byte g;
        final byte b;
        statusLights(byte r, byte g, byte b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }
    public void setIndicatorLights(statusLights s) {
        rgbLight.write(0x00, new byte[] {s.r, s.g, s.b});
    }
}
