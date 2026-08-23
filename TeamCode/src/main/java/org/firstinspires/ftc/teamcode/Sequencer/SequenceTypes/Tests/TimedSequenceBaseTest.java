package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.TimedSequenceBase;
import org.firstinspires.ftc.teamcode.Sequencer.SequencedOpMode;

@TeleOp(name = "TimedSequenceBaseTest")
public class TimedSequenceBaseTest extends SequencedOpMode {

    public TimedSequenceBase base = new TimedSequenceBase((a) -> {
        telemetry.addData("TimesSequenceBase",true);
    },1000);

    @Override
    public void onStart() {
        sequencer.addSequence(base);
    }


    @Override
    public void onLoop() {

    }
}