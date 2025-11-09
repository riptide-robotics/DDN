package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DummyClasses.EndgameServos;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "Zero Servos")
public class ZeroServos extends LinearOpMode {
    public static double liftServos = 0;
    private static double prevliftServos = 0;


    Servo liftServoL;
    Servo liftServoR;


    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Robot status", "successfully initiated");
        telemetry.update();

        liftServoL = hardwareMap.servo.get("liftServoLeft");
        liftServoR =  hardwareMap.servo.get("liftServoRight");
        liftServoR.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clear();
        while (opModeIsActive()) {
            zeroLiftServos();

            telemetry.update();
        }
    }
    public void zeroLiftServos() {
        if (prevliftServos == liftServos) return;
        prevliftServos = liftServos;
        liftServoL.setPosition(liftServos);
        liftServoR.setPosition(liftServos);

        telemetry.addData("Zeroed","Lift servos");
    }
}
