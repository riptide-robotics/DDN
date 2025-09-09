package org.firstinspires.ftc.teamcode.Modules.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Config
@TeleOp(name = "P2W computer")
public class GoBildaPinpoint extends LinearOpMode {

    GoBildaPinpointDriver odoComputer;
    double oldTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // INITIALIZATION
        odoComputer = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odoComputer.setOffsets(127, 299.72, DistanceUnit.MM);


        // Set what odo pods we using
        odoComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // UNCOMMENT THIS IF USING DIFFERENT PODS AND INPUT TICKS PER UNIT: odo.setEncoderResolution(13.26291192, DistanceUnit.MM);


        odoComputer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Resets position to 0,0,0 and recalibrates IMU
        odoComputer.resetPosAndIMU();


        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odoComputer.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odoComputer.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odoComputer.getDeviceVersion());
        telemetry.addData("Heading Scalar", odoComputer.getYawScalar());
        telemetry.update();

        waitForStart();
        resetRuntime();
        while (opModeIsActive()) {

            odoComputer.update();

            if (gamepad1.y){
                odoComputer.recalibrateIMU();
            }

            // Calculates time per each cycle and finds number of updates per seconds (frequency)
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            Pose2D pos = odoComputer.getPosition();

            telemetry.addLine("\n Positions and Heading \n")
                    .addData("X Position", pos.getX(DistanceUnit.MM))
                    .addData("Y Position", pos.getY(DistanceUnit.MM))
                    .addData("Orientation (Degrees)", Math.toDegrees(pos.getHeading(AngleUnit.DEGREES)));

            telemetry.addLine("\n Velocities \n")
                    .addData("X Velocity", odoComputer.getVelX(DistanceUnit.MM))
                    .addData("Y Velocity", odoComputer.getVelY(DistanceUnit.MM))
                    .addData("H Velocity", odoComputer.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

            telemetry.addData("Status", odoComputer.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", odoComputer.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();
        }
    }
}
