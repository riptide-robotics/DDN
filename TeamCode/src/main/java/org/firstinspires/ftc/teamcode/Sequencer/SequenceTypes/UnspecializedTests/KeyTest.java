package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.UnspecializedTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.TimedSequenceBase;
import org.firstinspires.ftc.teamcode.Sequencer.SequencedOpMode;
import org.firstinspires.ftc.teamcode.Sequencer.Sequencer;

@TeleOp(name = "KeyTest")
public class KeyTest extends SequencedOpMode {
    @Override
    public void onStart() {
        telemetry.addData("A",false);
    }

    @Override
    public void onLoop() {
        if (gamepad1.aWasPressed()) {
            sequencer.addSequence(new TimedSequenceBase((a) -> {
                telemetry.addData("A",true);
            }, 1000));
        }
    }
}
