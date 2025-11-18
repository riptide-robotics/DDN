package org.firstinspires.ftc.teamcode.Modules;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Indicator {
    private final HardwareMap hardwareMap;

    public void setIndicatorLights(byte r, byte g, byte b) {
        setIndicatorLights(
                String.format("%02X",r) + //i hope this is the right formatting
                String.format("%02X",g) +
                String.format("%02X",b)
        );
    };
    public void setIndicatorLights(String s) {
        throw new UnsupportedOperationException("While we wait...");
    }
    public void setIndicatorLights(int d) {
        setIndicatorLights(Integer.toHexString(d));
    }
    public Indicator(HardwareMap map) {
        this.hardwareMap = map;
    }
}
