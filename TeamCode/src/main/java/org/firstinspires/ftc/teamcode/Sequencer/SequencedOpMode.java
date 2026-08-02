package org.firstinspires.ftc.teamcode.Sequencer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * A variant of OpMode that has the sequencer system incorporated within. <br>
 *  Note that usage is not identical, relying on #onStart(), #onLoop(), and #onStop().
 *  */
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
