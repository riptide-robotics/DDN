package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.RepeatedSequence;
import org.firstinspires.ftc.teamcode.Sequencer.SequencedOpMode;

@TeleOp(name = "RepeatedSequenceTest")
public class RepeatedSequenceTest extends SequencedOpMode {


    public RepeatedSequence sequence = new RepeatedSequence((a) -> {
    telemetry.addData("Time",System.currentTimeMillis());
    },5000,1000);

    @Override
    public void onStart() {
        sequencer.addSequence(sequence);
    }

    @Override
    public void onLoop() {

    }
}
