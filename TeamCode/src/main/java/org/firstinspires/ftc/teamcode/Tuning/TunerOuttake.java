package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Outtake")
public class TunerOuttake extends LinearOpMode {
    public Robot robot;
    /**Keep between 0 and 1.*/
    private DcMotor motor;
    public static double outtakespeed = 0.8;

    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        motor = hardwareMap.dcMotor.get("Outtake");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            if (gamepad1.right_trigger > 0.1) motor.setPower(outtakespeed);
            else motor.setPower(0);
        }
    }

}
