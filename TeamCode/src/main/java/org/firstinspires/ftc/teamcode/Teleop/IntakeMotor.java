package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "tankDrive")
public class IntakeMotor extends LinearOpMode {
    DcMotor motor = hardwareMap.dcMotor.get("intakeMotor");
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Robot status", "succesfully initiated");
        telemetry.update();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clear();
        telemetry.addData("Robot status", "Started!");
        telemetry.update();

        while (opModeIsActive()) {
            spin();
        }
    }
    public void spin() {
        motor.setPower(1);
    }
}
