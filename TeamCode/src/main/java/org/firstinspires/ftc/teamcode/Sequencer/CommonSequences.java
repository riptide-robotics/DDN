package org.firstinspires.ftc.teamcode.Sequencer;

public enum CommonSequences {
    TEMPLATE(new RepeatedSequence(() -> {
        System.out.println("Template sequence");
    },50,100));

    public SequenceBase sequenceBase;
    private CommonSequences(SequenceBase sequence) {
        this.sequenceBase = sequence;
    }



}
