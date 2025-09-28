package org.firstinspires.ftc.teamcode.Modules;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.teamcode.Modules.Utils.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot;

// ----- READY TO TRANSFER ----- //

public class Drivetrain {

    ///////////////////////////////////////////////
    ////                                     /////
    ////              VARIABLES              /////
    ////                                     /////
    //////////////////////////////////////////////

    // -------- DRIVETRAIN MOTORS -------- //
    private final DcMotor frWheel, flWheel, brWheel, blWheel;
    private final IMU imu;
    private ElapsedTime timer;

    private volatile OdometryLocalizer robotPos3Wheel;
    private volatile GoBildaPinpointDriver robotPosPinpont;

    private boolean usingPinpoint = true;


    ///////////////////////////////////////////////
    ////                                     /////
    ////              FUNCTIONS              /////
    ////                                     /////
    //////////////////////////////////////////////

    // --------- INITIALIZATION --------- //

    public Drivetrain(HardwareMap hardwareMap) {

        frWheel = hardwareMap.dcMotor.get("frWheel");
        flWheel = hardwareMap.dcMotor.get("flWheel");
        brWheel = hardwareMap.dcMotor.get("brWheel");
        blWheel = hardwareMap.dcMotor.get("blWheel");

        blWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        brWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        brWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        blWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (usingPinpoint){
            robotPosPinpont = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
            robotPosPinpont.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            // UNCOMMENT THIS IF USING DIFFERENT PODS AND INPUT TICKS PER UNIT: odo.setEncoderResolution(13.26291192, DistanceUnit.MM);


            robotPosPinpont.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        }else {
            robotPos3Wheel = new OdometryLocalizer(blWheel, brWheel, flWheel, 10);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.resetYaw();
        imu.initialize(parameters);

    }

    // ----------- START/STOP ----------- //

    public void resetImu() {
        imu.resetYaw();
    }

    public double getRobotHeading(AngleUnit unit) {
        return imu.getRobotYawPitchRollAngles().getYaw(unit); // heading of bot in radians
    }

    // ------------ SETTERS ------------ //


    public void setWheelPowers(double flWheelPower, double frWheelPower, double brWheelPower, double blWheelPower) {
        frWheel.setPower(frWheelPower);
        flWheel.setPower(flWheelPower);
        brWheel.setPower(brWheelPower);
        blWheel.setPower(blWheelPower);
    }

    // ------------ GETTERS ------------ //
    public EditablePose2D getCurrPos() {
        if (usingPinpoint){
            return robotPosPinpont.getCurrPos();
        } else {
            return robotPos3Wheel.getCurrPos();
        }
    }

    public void startOdometry(boolean isPinPoint) {
        if (!isPinPoint) {
            Thread localizer = new Thread(robotPos3Wheel);
            localizer.start();
        }

        if (isPinPoint){
            // Resets position to 0,0,0 and recalibrates IMU
            robotPosPinpont.resetPosAndIMU();
        }

    }

    public OdometryLocalizer getRobotPos() {
        if(usingPinpoint) { throw new RuntimeException("Using Pinpoint, don't use this.");}

        return robotPos3Wheel;
    }
}