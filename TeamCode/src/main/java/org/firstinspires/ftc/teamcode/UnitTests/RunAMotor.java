package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Config
@TeleOp(name="Run a Motor")
public class RunAMotor extends LinearOpMode {

    DcMotor motor;
    public  static double power;

    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * * * * * * * * * * * * * * *
         * INITIALIZATION
         * * * * * * * * * * * * * * *
         */

        motor = hardwareMap.dcMotor.get("intakeMotor");

        motor.setDirection(DcMotor.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Robot status", "succesfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // * * * * * * * * * * * * * * *
        // * Start button clicked
        // * * * * * * * * * * * * * * *

        telemetry.clear();
        /*
         * * * * * * * * * * * * * * *
         * LOOP
         * * * * * * * * * * * * * * *
         */

        while(opModeIsActive()) {


            if(gamepad1.y){
                motor.setPower(power);
            }else{
                motor.setPower(0);
            }

            telemetry.update();
        }
    }
}
