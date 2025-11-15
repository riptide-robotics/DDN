package org.firstinspires.ftc.teamcode.Modules;

import static org.firstinspires.ftc.teamcode.riptideUtil.FORWARD_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.FORWARD_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.FORWARD_KP;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.teamcode.Modules.Utils.GoBildaPinpointDriver;

// ----- READY TO TRANSFER ----- //

public class Drivetrain {

    ///////////////////////////////////////////////
    ////                                     /////
    ////              VARIABLES              /////
    ////                                     /////
    //////////////////////////////////////////////

    // -------- DRIVETRAIN MOTORS -------- //
    private final DcMotor frWheel, flWheel, brWheel, blWheel;
    private final GoBildaPinpointDriver pinpoint;
    private final IMU imu;
    private ElapsedTime timer;

    private final OdometryLocalizer robotPos;
    private final double xOdoOffsetInInches = 0;
    private final double yOdoOffsetInInches = 0;

    // -------- AUTONOMOUS CONTROLLERS -------- //

    private final PIDController headingController = new PIDController(TURN_KP, TURN_KI, TURN_KD);
    private final PIDController forwardController = new PIDController(FORWARD_KP, FORWARD_KI, FORWARD_KD);


    //////////////////////////////////////////////
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
        flWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        frWheel.setDirection(DcMotorSimple.Direction.REVERSE);
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

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        pinpoint.setOffsets(xOdoOffsetInInches, yOdoOffsetInInches, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        robotPos = new OdometryLocalizer(pinpoint, 10);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.resetYaw();
        imu.initialize(parameters);

    }

    // ----------- START/STOP ----------- //

    public void resetImu() {
        imu.resetYaw();
    }

    public void PIDToPoint() {
        throw new UnsupportedOperationException("PIDToPoint not supported!");

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
        return robotPos.getCurrPos();
    }

    public void startOdometry() {
        Thread localizer = new Thread(robotPos);
        localizer.start();
    }

    public OdometryLocalizer getRobotPos() {
        return robotPos;
    }

    public double getRobotHeading(AngleUnit unit) {
        return imu.getRobotYawPitchRollAngles().getYaw(unit); // heading of bot in radians
    }
}