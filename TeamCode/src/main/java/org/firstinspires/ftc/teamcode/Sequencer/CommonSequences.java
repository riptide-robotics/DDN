package org.firstinspires.ftc.teamcode.Sequencer;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.*;

import java.util.concurrent.atomic.AtomicReference;

/**This enum is a way of storing sequences that are often run.*/
public enum CommonSequences {
    TEMPLATE((args) -> new RepeatedSequence((a) -> {
        System.out.println("Template sequence");
    },50,100)),

    ODOTEST((args) -> new DataWatcherSequence<>(
            () -> ((DcMotor) args[0]).getCurrentPosition(),
            Integer.class
    )),

    //not sure if this works
    VOLTAGEMOVEMENT((args) ->
        new RepeatedSequence((seq) -> {

            ((DcMotor) args[0]).setPower(0.8);
            ((DcMotor) args[1]).setPower(0.8);

            if (seq.values.get(0, Integer.class) > 10) seq.end = true;

        },0,500));







    private final ExecutableSequence executableSequence;
    CommonSequences(ExecutableSequence sequence) {
        this.executableSequence = sequence;
    }

    @FunctionalInterface
    private interface ExecutableSequence {
        SequenceBase run(Object[] args);
    }
    public SequenceBase getSequence(Object... args) {
        return executableSequence.run(args);
    }
}
