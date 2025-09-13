package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class AprilTagCustomDatabase {
    public static AprilTagLibrary getLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(
                        20,
                        "Blue Goal",
                        6.5,
                        //new VectorF(),
                        DistanceUnit.INCH
                        //Quaternion.identityQuaternion()
                )
                .addTag(
                        24,
                        "Red Goal",
                        6.5,
                        //new VectorF(),
                        DistanceUnit.INCH
                        //Quaternion.identityQuaternion()
                )
                .build();
    }
}
