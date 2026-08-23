package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.LinkedSequence;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.RepeatedSequence;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.TimedSequenceBase;
import org.firstinspires.ftc.teamcode.Sequencer.SequencedOpMode;

@TeleOp(name = "LinkedSequenceTest")
public class LinkedSequenceTest extends SequencedOpMode {

    boolean timedrepeat = false;
    boolean linkedrepeat = false;
    RepeatedSequence timed = new RepeatedSequence((a) -> {
        telemetry.addData("RepeatedSequence",timedrepeat = !timedrepeat);
    },3000,3000);

    LinkedSequence linked = new LinkedSequence((a) -> {
        telemetry.addData("LinkedSequence",linkedrepeat = !linkedrepeat);
    }, timed);


    @Override
    public void onStart() {
        telemetry.addData("TimedSequence",false);
        telemetry.addData("LinkedSequence",false);
        sequencer.addSequence(timed);
        sequencer.addSequence(linked);
    }

    @Override
    public void onLoop() {

    }
}
