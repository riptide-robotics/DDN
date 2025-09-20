package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import android.support.v4.app.INotificationSideChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;

/**
 * Creates a trapezoidal motion profile for 2 dimensions.
 * Usage guide in the documentation on discord
 */
public class TrapezoidalMotionProfile2D {

    private double maxA; // Maximum acceleration
    private double maxV; // Maximum velocity

    // Woah lookie here at Aaron's AI documentation o.o
    // Although I could have used more documentation so I wrote in a lot more.
    // All timing/distance values are now computed using the absolute value of the distance,
    // and then we reapply the sign at the end.
    private double dta;        // Time to reach max velocity (maxV) (always positive)
    private double effMaxV;    // Effective max velocity (can be lower than maxV if distance is short)
    private double accelDist;  // Distance covered during acceleration (absolute value)
    private double cruiseDist; // Distance covered during cruising (absolute value)
    private double cruiseTime; // Time spent cruising (always positive)
    private double decelTime;  // Start time of deceleration phase (always positive)
    private double totalTime;  // Total time for the entire motion profile (always positive)
    private double pathAngle;  // Angle of the path, in radians
    private EditablePose2D startPos; // start position

    public TrapezoidalMotionProfile2D(double maxA, double maxV) {
        if (maxA <= 0 || maxV <= 0) {
            throw new IllegalArgumentException("maxA and maxV must be > 0");
        }

        this.maxA = maxA;
        this.maxV = maxV;
    }

    public void setProfile(double mA, double mV) {
        if (maxA <= 0 || maxV <= 0) {
            throw new IllegalArgumentException("maxA and maxV must be > 0");
        }
        maxA = mA;
        maxV = mV;
    }

    // Calculate motion profile based on a goal and a current position.
    // This version calculates using the absolute distance and then re-applies the direction.

    /**
     * Creates the accelerate/decelerate portions of the path given. Most Crucially, this version stores the direction of travel.
     * @param goal End distance
     * @param currPos current position
     */
    public void calculateProfile(EditablePose2D goal, EditablePose2D currPos) {
        startPos = currPos;
        double dx = goal.getX(DistanceUnit.INCH) - currPos.getX(DistanceUnit.INCH);
        double dy = goal.getY(DistanceUnit.INCH) - currPos.getY(DistanceUnit.INCH);
        double distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
        pathAngle = Math.atan2(dy, dx);

        if (distance == 0) {
            dta = effMaxV = accelDist = cruiseDist = cruiseTime = decelTime = totalTime = 0.0;
            pathAngle = 0.0;
            return;
        }

        dta = maxV / maxA; // v = at therefore t = v/a
        double halfDistance = distance / 2; // test var
        accelDist = 0.5 * maxA * Math.pow(dta, 2); // ∆x = 1/2 a∆t^2 (absolute value)

        effMaxV = maxV; // Will stay this way unless proven could be better (Yeah...)
        if (accelDist > halfDistance) { // In case accelDist is too big. (Cuz the largest it can be is right at the midpoint.)
            dta = Math.sqrt(2 * halfDistance / maxA); // fix dta in this case (otherwise it's not gon' work)
            effMaxV = dta * maxA; // fix effMaxV (Just because it'll help-) (yeah, yes.)
            accelDist = 0.5 * maxA * Math.pow(dta, 2); // re-solve accelDist with new dta
        }

        cruiseDist = distance - 2 * accelDist; // Calculate the distance the robot spends at a constant velocity (cruiseDist)
        cruiseTime = cruiseDist / effMaxV; // Since effMaxV is the max in this profile. And cuz x = vt therefore t = x/v.
        decelTime = dta + cruiseTime; // decelTime is WHEN it starts decelerating. That's after acceleration and cruising.
        totalTime = 2 * dta + cruiseTime; // or dta + cruiseTime + dta (Honestly who gives a fuck so)

    }

    // Convenience overload if you just have a distance (assumes starting at 0)
//    public void calculateProfile(double distance) {
//        calculateProfile(distance, 0);
//    }

    /**
     * @param time current time, in seconds
     * @return expectedPosition from Profile
     */
    public EditablePose2D getExpectedPosition(double time) {
        double pos; // computed position (absolute value)

        // If time exceeds the total duration, return the target distance (absolute value)
        if (time > totalTime) {
            pos = 2 * accelDist + cruiseDist; // This is the full distance
        }
        // return expected position depending on the phase
        else if (time < dta) { // Acceleration phase
            pos = 0.5 * maxA * Math.pow(time, 2); // ∆x = 0.5at^2
        } else if (time < decelTime) { // Cruise phase (decelTime is WHEN deceleration starts)
            pos = accelDist + effMaxV * (time - dta); // The triangle plus the rectangle (draw the trapezoid if ur confused (in ur head lol))
        } else { // Deceleration phase
            pos = accelDist + cruiseDist + effMaxV * (time - decelTime) - 0.5 * maxA * Math.pow(time - decelTime, 2); // Draw it
        }

        double expPos1d = pos;
        double expX = startPos.getX(DistanceUnit.INCH) + expPos1d * Math.cos(pathAngle);
        double expY = startPos.getY(DistanceUnit.INCH) + expPos1d * Math.sin(pathAngle);

        return new EditablePose2D(expX, expY, startPos.getH(), DistanceUnit.INCH);

        //
        //       _________________________
        //      /|                   |\  | look at this beautiful finished rectangle on the right
        //     / |                   | \ | of this unfinished trapezoid
        //    /  |                   |  \| I am obsessed with diagrams I know
        //   /   |                   |   | And ihdgaf (I honestly don't give a fuck).
        //  /    |                   |   |
        // /_____|___________________|___|
        //
    }
}
