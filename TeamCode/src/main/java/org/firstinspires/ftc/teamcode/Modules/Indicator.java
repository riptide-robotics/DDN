package org.firstinspires.ftc.teamcode.Modules;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Indicator {
    private final HardwareMap hardwareMap;
    private static final String acceptedChars = "0123456789ABCDEF";

    public enum statusLights {
        //yeah just random shit so this actually has something. Add to this when you want.
        EMPTY("000000"),
        SEMI_OPEN("000000"),
        SEMI_OPEN_AND_NONE_REQUESTED("000000"),
        FULL_SPINDEXER("000000"),
        FULL_SPINDEXER_AND_NONE_REQUESTED("000000");

        String color;
        statusLights(String color) {
            ensureColor(color);
            this.color = color;
        }


    }

    public void setIndicatorLights(byte r, byte g, byte b) {
        setIndicatorLights(
                String.format("%02X",r) + //i hope this is the right formatting
                String.format("%02X",g) +
                String.format("%02X",b)
        );
    };
    public void setIndicatorLights(String s) {
        ensureColor(s);
        throw new UnsupportedOperationException("While we wait...");
    }
    public void setIndicatorLights(int d) {
        setIndicatorLights(Integer.toHexString(d));
    }
    public void setIndicatorLights(statusLights s) {
        setIndicatorLights(s.color);
    }
    public Indicator(HardwareMap map) {
        this.hardwareMap = map;
    }

    private static void ensureColor(String color) {
        if (color.length() != 6) throw new RuntimeException("Use a color with a proper length, this isn't JavaScript.");

        for (char c : color.toCharArray())
            if (acceptedChars.indexOf(c) == -1) throw new RuntimeException("Use hexadecimal!");
    }
}
