package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Modules.Camera;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Slides;
import org.firstinspires.ftc.teamcode.Modules.TurnTable;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Robot {

    HardwareMap hardwareMap;

    Drivetrain drivetrain;
    Camera camera;

    TurnTable table;

    public Robot (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        drivetrain = new Drivetrain(hardwareMap);
        camera = new Camera(hardwareMap);
        table = new TurnTable(2, 0, 0);
    }

    public Drivetrain getDrivetrain(){
        return drivetrain;
    }
    public Camera getCamera() {
        return camera;
    }
    public TurnTable getTable() {return table;}

}
