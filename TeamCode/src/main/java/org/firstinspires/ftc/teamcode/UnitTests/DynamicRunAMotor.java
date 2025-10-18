package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="Dynamical Run A Motor", group="Unit Tests")
public class DynamicRunAMotor extends LinearOpMode {

    DcMotor motor;
    public static double motorPower = 1;
    public static String motorName = "intakeMotor";
    private String previousMotorName = "";

    public static boolean direction = true;
    private void motorInit(String name){
        motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }


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
                motor.setPower(motorPower);
            }else{
                motor.setPower(0);
            }

            telemetry.update();

            if(!previousMotorName.equals(motorName)){
                previousMotorName = motorName;
                motorInit(motorName);
            }
        }
    }
}
