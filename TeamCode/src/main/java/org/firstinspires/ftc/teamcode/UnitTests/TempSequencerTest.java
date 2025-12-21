package org.firstinspires.ftc.teamcode.UnitTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Utils.Sequencer;

import java.util.UUID;

/**This thing is only here to circumvent the bot not really existing. Delete when it does.*/
@TeleOp(name = "TempSequencerTest")
public class TempSequencerTest extends LinearOpMode {
    Sequencer s;
    double testedValue = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        s = new Sequencer(null,telemetry); //drive train does not yet exist
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            telemetry.addData("test",testedValue);
            if (gamepad1.aWasPressed()) {
                s.addRepeatingAction(() -> {
                    testedValue++;
                }, 4, UUID.randomUUID().toString());
            }

            s.loop();
            telemetry.update();
        }
    }
}
