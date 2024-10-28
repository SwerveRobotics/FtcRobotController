package org.firstinspires.ftc.team417.distance;

public class DistanceSensorInfo {
    private final double xOffset;
    private final double yOffset;
    private final double thetaOffset;

    public DistanceSensorInfo(double xOffset, double yOffset, double thetaOffset) {
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.thetaOffset = thetaOffset;
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

    @Override
    public String toString() {
        return "DistanceSensorInfo{" +
                "xOffset=" + xOffset +
                ", yOffset=" + yOffset +
                ", thetaOffset=" + thetaOffset +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof DistanceSensorInfo)) return false;

        DistanceSensorInfo that = (DistanceSensorInfo) o;

        return Double.compare(that.xOffset, xOffset) == 0 &&
                Double.compare(that.yOffset, yOffset) == 0 &&
                Double.compare(that.thetaOffset, thetaOffset) == 0;
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
        return result;
    }
}

