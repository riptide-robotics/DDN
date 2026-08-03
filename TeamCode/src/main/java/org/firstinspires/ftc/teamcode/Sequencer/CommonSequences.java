package org.firstinspires.ftc.teamcode.Sequencer;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.DataWatcherSequence;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.LinkedSequence;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.RepeatedSequence;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.SequenceBase;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.TimedSequenceBase;

/**This enum is a way of storing sequences that are often run.*/
public enum CommonSequences {
    TEMPLATE((args) -> new RepeatedSequence(() -> {
        System.out.println("Template sequence");
    },50,100)),

    ODOTEST((args) -> new DataWatcherSequence<>(
            () -> ((DcMotor) args[0]).getCurrentPosition(),
            Integer.class
    ));


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
