package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.OrderedSequence;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.RepeatedSequence;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.TimedSequenceBase;
import org.firstinspires.ftc.teamcode.Sequencer.SequencedOpMode;

@TeleOp(name = "OrderedSequenceTest")
public class OrderedSequenceTest extends SequencedOpMode {


    boolean repeat = false;
    public TimedSequenceBase base1 = new TimedSequenceBase((a) -> {
        telemetry.addData("base1", true);
    },1000);

    public TimedSequenceBase base2 = new TimedSequenceBase((a) -> {
        telemetry.addData("base2", true);
    },1000);

    public TimedSequenceBase base3 = new TimedSequenceBase((a) -> {
        telemetry.addData("base3", true);
    },1000);

    public RepeatedSequence repeatseq = new RepeatedSequence((a) -> {
        telemetry.addData("repeat",repeat = !repeat);
    },1000,500);

    public TimedSequenceBase unexecutable = new TimedSequenceBase((a) -> {
        telemetry.addData("unexecutable", true);
    },1000);


    public OrderedSequence tested = new OrderedSequence(base1,base2,base3,repeatseq,unexecutable);
    @Override
    public void onStart() {
        sequencer.addSequence(tested);
    }

    @Override
    public void onLoop() {

    }
}
