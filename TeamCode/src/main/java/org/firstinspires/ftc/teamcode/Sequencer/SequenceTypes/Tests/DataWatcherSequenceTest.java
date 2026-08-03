package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.Utils.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Sequencer.CommonSequences;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.DataWatcherSequence;
import org.firstinspires.ftc.teamcode.Sequencer.SequencedOpMode;

@TeleOp(name = "DataWatcherSequenceTest")
public class DataWatcherSequenceTest extends SequencedOpMode {

    DataWatcherSequence<Integer> dws;
    private DcMotor odo;
    @Override
    public void onStart() {

        odo = hardwareMap.dcMotor.get("pinpoint");

        dws = new DataWatcherSequence<>(
                () -> odo.getCurrentPosition(),
                Integer.class
        );

        this.sequencer.addSequence(CommonSequences.ODOTEST.getSequence(hardwareMap));


    }
    @Override
    public void onLoop() {
        telemetry.addData("Odo Pod Position", dws.get());
    }
}
