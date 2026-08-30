package org.firstinspires.ftc.teamcode.UnitTests;

// DO NOT TRANSFER

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Indicator;

import java.util.Arrays;


//---------------------------------------------------------------//
// This is not an accurate representation of the latest version. //
//---------------------------------------------------------------//


@Config
@TeleOp(name = "IndicatorScanTest")
public class IndicatorScanTest extends LinearOpMode {

    public static String slot1_ = "p";
    public static String slot2_ = "p";
    public static String slot3_ = "g";
    public static String motiforder1_ = "g";
    public static String motiforder2_ = "p";
    public static String motiforder3_ = "p";
    public static int artifactsInTrough_ = 0;

    private static char slot1 = 'p';
    private static char slot2 = 'p';
    private static char slot3 = 'g';
    private static char motiforder1 = 'g';
    private static char motiforder2 = 'p';
    private static char motiforder3 = 'p';
    private static byte artifactsInTrough = 0;

    Indicator indicator;

    @Override
    public void runOpMode() throws InterruptedException {
        indicator = new Indicator(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            iwishthatthedashboardsupportedchars();
            setStatus(artifactsInTrough);
        }
    }
    //Methods:
    public void setStatus(char colorRequested) {
        char[] order = new char[] {slot1,slot2,slot3};

        boolean contains =
                order[0] == colorRequested ||
                        order[1] == colorRequested ||
                        order[2] == colorRequested ;

        if (colorRequested == ' ') contains = true;

        byte amount = (byte) (
                (order[0] != ' ' ? 1:0) +
                        (order[1] != ' ' ? 1:0) +
                        (order[2] != ' ' ? 1:0) );

        if (amount == 0) indicator.setStatusColor(Indicator.statusLights.EMPTY);

        if ((amount == 1 || amount == 2) && contains) indicator.setStatusColor(Indicator.statusLights.SEMI_OPEN);
        if ((amount == 1 || amount == 2) && !contains) indicator.setStatusColor(Indicator.statusLights.SEMI_OPEN_WITHOUT_MOTIF);

        if (amount == 3 && contains) indicator.setStatusColor(Indicator.statusLights.FULL_SPINDEXER);
        if (amount == 3 && !contains) indicator.setStatusColor(Indicator.statusLights.FULL_SPINDEXER_WITHOUT_MOTIF);
    }

    /***/
    public void setStatus(byte artifactsInTrough) {
        char[] motifOrder = {motiforder1,motiforder2,motiforder3};
//        char[] badChar = {'b','b','b'};
//
//        if (Arrays.equals(motifOrder, badChar)) motifOrder = camera.scanMotifOrder(); //is there a problem
//        if (!Arrays.equals(motifOrder, badChar)) return; //is there still a problem

        setStatus(motifOrder[artifactsInTrough % 3]);
    }

    public void iwishthatthedashboardsupportedchars() {
        slot1=slot1_.charAt(0);
        slot2=slot2_.charAt(0);
        slot3=slot3_.charAt(0);
        motiforder1=motiforder1_.charAt(0);
        motiforder2=motiforder2_.charAt(0);
        motiforder3=motiforder3_.charAt(0);
        artifactsInTrough= (byte) artifactsInTrough_;
    }

}
