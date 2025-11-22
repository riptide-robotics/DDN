package org.firstinspires.ftc.teamcode.Autonomous.Utils.Trajectories;

import java.util.ArrayList;

/**
 *
 */
public class Trajectory {

    ArrayList<PathSample> path;

    public Trajectory(ArrayList<PathSample> samples) {
        this.path = samples;
    }

    public ArrayList<PathSample> getPath() {
        return this.path;
    }

    /**
     * A single baked point on the trajectory.
     */
    public static class PathSample {
        public final double time;      // seconds
        public final double x;         // field frame
        public final double y;         // field frame
        public final double heading;   // deg, field-centric

        // optional but super useful for feedforward
        public final double vx;        // in/s in field frame
        public final double vy;        // in/s in field frame
        public final double omega;     // deg/s

        public PathSample(
                double time,
                double x, double y, double heading,
                double vx, double vy, double omega
        ) {
            this.time = time;
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
        }
    }


    public PathSample getExpectedPosition(double time) {
        if (path == null || path.isEmpty()) {
            throw new IllegalStateException("Trajectory has no samples");
        }

        if (path.size() == 1) {
            return path.get(0);
        }

        if (time <= path.get(0).time) {
            return path.get(0);
        }
        if (time >= path.get(path.size() - 1).time) {
            return path.get(path.size() - 1);
        }

        int small = 0;
        int big = path.size() - 1;

        while (big - small > 1) {
            int mid = (big + small) / 2;
            PathSample s = path.get(mid);

            if (s.time < time) {
                small = mid;
            } else {
                big = mid;
            }
        }

        PathSample lower = path.get(small);
        PathSample upper = path.get(big);
        double percent = (time - lower.time) / (upper.time - lower.time);
        return new PathSample(
                time,
                lerp(lower.x, upper.x, percent),
                lerp(lower.y, upper.y, percent),
                lerpAngle(lower.heading, upper.heading, percent),
                lerp(lower.vx, upper.vx, percent),
                lerp(lower.vy, upper.vy, percent),
                lerp(lower.omega, upper.omega, percent)
        );
    }

    public double lerp(double a, double b, double percent) {
        return (1 - percent) * a + percent * b;
    }

    // chooses the shortest direction to lerp angles are always between 0 and 360.
    public double lerpAngle(double a, double b, double percent) {
        double diff = b - a;

        if (diff > 180) {
            diff -= 360;
        } else if (diff < -180) {
            diff += 360;
        }

        double result = a + diff * percent;

        // wrap to [0, 360)
        if (result < 0) result += 360;
        else if (result >= 360) result -= 360;

        return result;
    }


}
