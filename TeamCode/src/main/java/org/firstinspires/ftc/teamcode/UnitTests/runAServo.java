package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "run a servo")
public class runAServo extends LinearOpMode {

    Servo s;
    public  static double pos = 0;
    @Override
    public void runOpMode() throws  InterruptedException{
        s = hardwareMap.get(Servo.class, "bootkicker");
        s.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            s.setPosition(gamepad1.left_stick_x);
        }
    }
}
