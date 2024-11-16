package org.firstinspires.ftc.team417.distance;

import com.acmerobotics.roadrunner.Pose2d;

// A data class for keeping track of where distance sensors are relative to the center of the robot.
public class DistanceSensorInfo {
    private final double xOffset;
    private final double yOffset;
    private final double thetaOffset;
    private final Pose2d pose;

    public DistanceSensorInfo(double xOffset, double yOffset, double thetaOffset) {
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.thetaOffset = thetaOffset;
        this.pose = new Pose2d(xOffset, yOffset, -thetaOffset);
        // Negative because Roadrunner uses counterclockwise is positive, while this class uses clockwise is positive
    }

    public double getXOffset() {
        return xOffset;
    }

    public double getYOffset() {
        return yOffset;
    }

    public double getThetaOffset() {
        return thetaOffset;
    }

    public Pose2d getPose() {
        return pose;
    }

    @Override
    public String toString() {
        return "DistanceSensorInfo{" +
                "xOffset=" + xOffset +
                ", yOffset=" + yOffset +
                ", thetaOffset=" + thetaOffset +
                ", pose=" + pose +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof DistanceSensorInfo)) return false;

        DistanceSensorInfo that = (DistanceSensorInfo) o;

        return Double.compare(that.xOffset, xOffset) == 0 &&
                Double.compare(that.yOffset, yOffset) == 0 &&
                Double.compare(that.thetaOffset, thetaOffset) == 0 &&
                that.pose.equals(pose);
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = Double.doubleToLongBits(xOffset);
        result = (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(yOffset);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(thetaOffset);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = pose.hashCode();
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }
}

