package org.firstinspires.ftc.teamcode.UnitTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Indicator;
import org.firstinspires.ftc.teamcode.Modules.Utils.Sequencer;

@TeleOp
public class IndicatorTest extends LinearOpMode {
    Indicator indicator;
    Sequencer seq;
    byte currentState = 0;
    double prevTime;


    @Override
    public void runOpMode() throws InterruptedException {
        indicator = new Indicator(hardwareMap);
        seq = new Sequencer(null);

        waitForStart();
        if (isStopRequested()) return;

        prevTime = System.currentTimeMillis();

        while (opModeIsActive()) {
           telemetry.addData("state", currentState);
           telemetry.update();
           if (currentState == (byte) 0) indicator.setStatusColor(Indicator.statusLights.EMPTY);
           if (currentState == (byte) 1) indicator.setStatusColor(Indicator.statusLights.SEMI_OPEN);
           if (currentState == (byte) 2) indicator.setStatusColor(Indicator.statusLights.SEMI_OPEN_AND_NONE_REQUESTED);
           if (currentState == (byte) 3) indicator.setStatusColor(Indicator.statusLights.FULL_SPINDEXER);
           if (currentState == (byte) 4) indicator.setStatusColor(Indicator.statusLights.FULL_SPINDEXER_AND_NONE_REQUESTED);
         if (System.currentTimeMillis() - prevTime > 1000) {
             prevTime = System.currentTimeMillis();
             currentState++;
             if (currentState > 4) currentState = 0;
         }
        }

    }
}
