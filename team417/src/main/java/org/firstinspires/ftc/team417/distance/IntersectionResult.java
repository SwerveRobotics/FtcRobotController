package org.firstinspires.ftc.team417.distance;

// Combines a FieldSide enum and a distance double
public class IntersectionResult {
    FieldSide side;
    double distance;  // -1 if no intersection

    public IntersectionResult(FieldSide side, double distance) {
        this.side = side;
        this.distance = distance;
    }

    @Override
    public String toString() {
        return String.format("Side: %s, Distance: %.2f", side, distance);
    }
}