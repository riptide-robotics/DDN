package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp(name="run2Servos", group="UnitTests")
public class Run2Servos extends LinearOpMode {

    Servo s1;
    Servo s2;

    public static double s1Pos = 0;
    public static double s2Pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * * * * * * * * * * * * * * *
         * INITIALIZATION
         * * * * * * * * * * * * * * *
         */

        s1 = hardwareMap.servo.get("s1");
        s2 =  hardwareMap.servo.get("s2");
        s2.setDirection(Servo.Direction.REVERSE);

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
            s1.setPosition(s1Pos);
            s2.setPosition(s2Pos);


            telemetry.update();
        }
    }
}
