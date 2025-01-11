package org.firstinspires.ftc.team417.liveView;

public class CromaRun {
    public int x;
    public int y;
    public int length;
    public Colors color;

    public enum Colors  {
        YELLOW(0),
        BLUE(1),
        RED(2),
        OTHER(3);

        private final int value;

        Colors(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public CromaRun(int x, int y, int length, Colors color) {
        this.x = x;
        this.y = y;
        this.length = length;
        this.color = color;
    }
}
