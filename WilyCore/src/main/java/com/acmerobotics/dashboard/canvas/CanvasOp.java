package com.acmerobotics.dashboard.canvas;

public abstract class CanvasOp {

    public enum Type {
        GRID,
        TRANSLATE,
        ROTATION,
        SCALE,
        ALPHA,
        CIRCLE,
        POLYGON,
        POLYLINE,
        SPLINE,
        STROKE,
        FILL,
        STROKE_WIDTH,
        TEXT,
        IMAGE
    }

    private Type type;
    public CanvasOp(Type type) {
        this.type = type;
    }
}
