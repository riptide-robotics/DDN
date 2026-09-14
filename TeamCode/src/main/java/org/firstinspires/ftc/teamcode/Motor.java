package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;
import java.util.Objects;

@TeleOp (name = "name1")
public class Motor extends LinearOpMode {
    DcMotor motor;
    Servo servo;

    @Override
    public void runOpMode(){

        telemetry.addData("Motors", hardwareMap.dcMotor.entrySet());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;


        for (Map.Entry<String, DcMotor> entry: hardwareMap.dcMotor.entrySet()) {
            if (Objects.equals(entry.getKey(), "motor")) motor = entry.getValue();
        }
        for (Map.Entry<String, Servo> entry: hardwareMap.servo.entrySet()) {
            if (Objects.equals(entry.getKey(), "servo")) servo = entry.getValue();
        }
        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor.setPower(1);

            }
            else {
                motor.setPower(0);
            }
        }
    }
}
