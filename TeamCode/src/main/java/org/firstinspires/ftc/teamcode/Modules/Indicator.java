package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Placeholder;


@Placeholder(note = "Waiting on an RGB light")
public class Indicator {
    private final HardwareMap hardwareMap;
    private final Servo rgbLightServo;
    private final PwmControl rgbLight;
    public Indicator(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.rgbLightServo = hardwareMap.servo.get("indicator");
        this.rgbLight = (PwmControl) rgbLightServo;

        rgbLight.setPwmRange(new PwmControl.PwmRange(500,2500));
    }

    public enum statusLights {
        EMPTY(0.3),
        SEMI_OPEN(0.389),
        SEMI_OPEN_AND_NONE_REQUESTED(0.501),
        FULL_SPINDEXER(0.612),
        FULL_SPINDEXER_AND_NONE_REQUESTED(0.700);

       final double pos;
        statusLights(double pos) {
            this.pos = pos;
        }
    }
    public void setStatusColor(statusLights lights) {
        rgbLightServo.setPosition(lights.pos);
    }
}
