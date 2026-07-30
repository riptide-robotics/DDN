package org.firstinspires.ftc.teamcode.Sequencer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class SequencedOpMode extends LinearOpMode {

    Sequencer sequencer;
    @Override
    public void runOpMode() throws InterruptedException {
        sequencer = new Sequencer();

        onStart();

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {
            sequencer.iterate();
            onLoop();
        }
        onStop();
    }

    public abstract void onStart();
    public abstract void onLoop();
    public void onStop() {};
}
