package org.firstinspires.ftc.teamcode.Modules;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Placeholder;


public class Indicator {
    private final HardwareMap hardwareMap;
    private final Servo rgbLightServo;
    private final PwmControl rgbLight;
    /**Access Indicator.*/
    public Indicator(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.rgbLightServo = hardwareMap.servo.get("indicator");
        this.rgbLight = (PwmControl) rgbLightServo;

        rgbLight.setPwmRange(new PwmControl.PwmRange(500,2500));
    }

    /**Already defined colors to represent the status of the bot. Not final.*/
    public enum statusLights {
        EMPTY(0.301),
        SEMI_OPEN_WITHOUT_MOTIF(0.389),
        @Deprecated
        SEMI_OPEN_AND_NONE_REQUESTED(0.501),
        SEMI_OPEN(0.501),

        FULL_SPINDEXER(0.612),
        @Deprecated
        FULL_SPINDEXER_AND_NONE_REQUESTED(0.700),
        FULL_SPINDEXER_WITHOUT_MOTIF(0.900);


        /**The position the "servo" the RGB indicator is set to, in order to set it to a specific color.*/
       final double pos;
        statusLights(double pos) {
            this.pos = pos;
        }
    }
    /**Set status lights to values intended to display that of the bot. The colors are not final.*/
    public void setStatusColor(statusLights lights) {
        rgbLightServo.setPosition(lights.pos);
    }
    /**Apply a color using one of several templates.*/
    public void setRGB(IndColor c) {
        rgbLightServo.setPosition(c.getColor());
    }

    /**Set color without any safeguards. May result in the wrong color.*/
    public void setColor(double rotation) {
        rgbLightServo.setPosition(rotation);
    }
    /**This enum represents a range of colors that the indicator lights can be set to.*/
    public enum IndColor {
        RED(0.301),
        ORANGE(0.333),
        YELLOW(0.388),
        SAGE(0.444),
        GREEN(0.5),
        AZURE(0.555),
        BLUE(0.611),
        INDIGO(0.666),
        VIOLET(0.722),
        WHITE(0.9),
        NONE(0.1);
        private final double rot;
        public double getColor() {
            return rot;
        }
        IndColor(double rot) {
            this.rot = rot;
        }
    }
}
