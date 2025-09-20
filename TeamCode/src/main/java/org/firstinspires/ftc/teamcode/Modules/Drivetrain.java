package org.firstinspires.ftc.teamcode.Modules;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;

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

    private final OdometryLocalizer robotPos;

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

        robotPos = new OdometryLocalizer(blWheel, brWheel, flWheel, 10);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.resetYaw();
        imu.initialize(parameters);

    }

    // ----------- START/STOP ----------- //

    /**
     * This method resets the robot's Y angle to 0. The orientation will be relative to
     * that of the robot in its current position - it does not move the robot.
     * <p></p>
     * This is identical to retrieving the IMU of this (only possible through odd means)
     * and resetting the yaw there. However, it is recommended to use this.
     * */
    public void resetImu() {
        imu.resetYaw();
    }

    /**
     * Gets the orientation of the robot, with regard to AngleUnit (self explanatory).
     * It is recommended to stick to radians for this.
     * */
    public double getRobotHeading(AngleUnit unit) {
        return imu.getRobotYawPitchRollAngles().getYaw(unit); // heading of bot in radians
    }

    // ------------ SETTERS ------------ //

    /**
     * <p>
     *     This sets power to the wheels, effectively controlling their speed and direction.
     *     Use a negative number for moving backwards.
     * </p>
     * <p>
     *     The power ranges from -1 to 1, with 0 representing no movement, negative numbers
     *     representing moving backwards, and positive numbers representing moving forward.
     * </p>
     * @param flWheelPower front left wheel power
     * @param frWheelPower
     *
     */
    public void setWheelPowers(double flWheelPower, double frWheelPower, double brWheelPower, double blWheelPower) {
        frWheel.setPower(frWheelPower);
        flWheel.setPower(flWheelPower);
        brWheel.setPower(brWheelPower);
        blWheel.setPower(blWheelPower);
    }

    // ------------ GETTERS ------------ //
    /**
     * Gets the current position of the bot. Although it uses odometry, it is very capable
     * of being called many times very quickly.
     * */
    public EditablePose2D getCurrPos() {
        return robotPos.getCurrPos();
    }
    /**
     * Starts the whole odometry process. This puts it in a separate thread, so be
     * careful when using this.
     * */
    public void startOdometry() {
        Thread localizer = new Thread(robotPos);
        localizer.start();
    }
    /**
     * Gets the position of the robot. A getter for its OdometryLocalizer.
     * <p>There is no setter.</p>
     * */
    public OdometryLocalizer getRobotPos() {
        return robotPos;
    }
}