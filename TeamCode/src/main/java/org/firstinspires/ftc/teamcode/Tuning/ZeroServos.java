package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="TuneLift", group="tuning")
public class ZeroServos extends LinearOpMode {

   Servo liftLeft;
   Servo liftRight;

   public static double pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * * * * * * * * * * * * * * *
         * INITIALIZATION
         * * * * * * * * * * * * * * *
         */
        liftLeft = hardwareMap.get(Servo.class, "liftServoLeft");
        liftRight = hardwareMap.get(Servo.class, "liftServoRight");

        liftLeft.setDirection(Servo.Direction.REVERSE);
        liftRight.setDirection(Servo.Direction.FORWARD);


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
            liftLeft.setPosition(pos);
            liftRight.setPosition(pos);
        }
    }
}
