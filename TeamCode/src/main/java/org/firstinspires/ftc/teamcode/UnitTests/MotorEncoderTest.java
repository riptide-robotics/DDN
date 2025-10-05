package org.firstinspires.ftc.teamcode.UnitTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Timer;

@TeleOp(name="Motor Encoder Test", group="Unit Test")
public class MotorEncoderTest extends LinearOpMode {
    DcMotor motor;
    double prevpos = 0;
    Timer elapsed_time = new Timer();

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("flWheel");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean a_pressed = false;
        boolean a_debounce = false;

        boolean b_pressed = false;
        boolean b_debounce = false;

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a) {
                b_pressed = false;
                if(!a_debounce) {
                    a_debounce = true;
                    a_pressed = !a_pressed;
                }
            } else {
                a_debounce = false;
            }

            if(gamepad1.b) {
                if(!b_debounce) {
                    b_debounce = true;
                    b_pressed = true;
                }
            } else {
                b_debounce = false;
            }

            if(!a_pressed && gamepad1.left_stick_x!=0) {
                b_pressed = false;
            }

            if (!b_pressed) {
                if (!a_pressed) {
                    motor.setPower(gamepad1.left_stick_x);
                } else {
                    motor.setPower(0.2);
                }
            } else {
                motor.setPower(0);
            }

            telemetry.addData("Current Power: ", motor.getPower());
            telemetry.addData("Current Position: ", motor.getCurrentPosition());
            telemetry.addData("Motor Revolutions: ", (double) motor.getCurrentPosition() / 28);
            telemetry.update();

            prevpos = motor.getCurrentPosition();
        }
    }
}
