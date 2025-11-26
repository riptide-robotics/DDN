package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "run a servo")
public class runAServo extends LinearOpMode {
    Robot robot;
    public  static double pos = 0;
    @Override
    public void runOpMode() throws  InterruptedException{
        robot = new Robot(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            robot.getIntake().spindexPos(pos);
        }
    }
}
