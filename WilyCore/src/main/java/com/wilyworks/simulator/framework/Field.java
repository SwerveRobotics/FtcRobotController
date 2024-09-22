package com.wilyworks.simulator.framework;


import static java.lang.Thread.currentThread;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.wilyworks.common.WilyWorks;
import com.wilyworks.simulator.WilyCore;
import com.wilyworks.simulator.helpers.Point;

import java.awt.AlphaComposite;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.GraphicsConfiguration;
import java.awt.GraphicsEnvironment;
import java.awt.Image;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.Transparency;
import java.awt.geom.AffineTransform;
import java.awt.geom.GeneralPath;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.InputStream;

import javax.imageio.ImageIO;

/**
 * Field view manager.
 */
public class Field {
    // Make the field view 720x720 pixels but inset the field surface so that there's padding
    // all around it:
    static final int FIELD_VIEW_DIMENSION = 500;
    static final int FIELD_SURFACE_DIMENSION = 480;

    // These are derived from the above to describe the field rendering:
    static final int FIELD_INSET = (FIELD_VIEW_DIMENSION - FIELD_SURFACE_DIMENSION) / 2;
    static final Rectangle FIELD_VIEW = new Rectangle(0, 0, FIELD_VIEW_DIMENSION, FIELD_VIEW_DIMENSION);

    // Robot dimensions:
    static final int ROBOT_IMAGE_WIDTH = 128;
    static final int ROBOT_IMAGE_HEIGHT = 128;

    Simulation simulation;
    Image backgroundImage;
    Image compassImage;
    BufferedImage robotImage;

    public Field(Simulation simulation) {
        this.simulation = simulation;
        ClassLoader classLoader = currentThread().getContextClassLoader();

        InputStream compassStream = classLoader.getResourceAsStream(
                "background/misc/compass-rose-white-text.png");
        InputStream fieldStream = classLoader.getResourceAsStream(
                "background/season-2024-intothedeep/field-2024-juice-dark.png");

        try {
            if (compassStream != null) {
                compassImage = ImageIO.read(compassStream)
                        .getScaledInstance(150, 150, Image.SCALE_SMOOTH);
            }
            if (fieldStream != null) {
                backgroundImage = ImageIO.read(fieldStream)
                        .getScaledInstance(FIELD_SURFACE_DIMENSION, FIELD_SURFACE_DIMENSION, Image.SCALE_SMOOTH);
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        initializeRobotImage();
    }

    // Round to an integer:
    static int round(double value) {
        return (int) Math.round(value);
    }

    // Initialize the robot image bitmap:
    private void initializeRobotImage() {
        final int OPACITY = round(255 * 0.8);
        final double WHEEL_PADDING_X = 0.05;
        final double WHEEL_PADDING_Y = 0.05;
        final double WHEEL_WIDTH = 0.2;
        final double WHEEL_HEIGHT = 0.3;
        final double DIRECTION_LINE_WIDTH = 0.05;
        final double DIRECTION_LINE_HEIGHT = 0.4;

        GraphicsConfiguration config =
                GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice().getDefaultConfiguration();
        robotImage = config.createCompatibleImage(ROBOT_IMAGE_WIDTH, ROBOT_IMAGE_HEIGHT, Transparency.TRANSLUCENT);

        Graphics2D g = robotImage.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);

        // Draw the body:
        g.setColor(new Color(0xe5, 0x3e, 0x3d, OPACITY));
        g.fillRect(0, 0, ROBOT_IMAGE_WIDTH, ROBOT_IMAGE_HEIGHT);

        // Draw the wheels:
        g.setColor(new Color(0x74, 0x2a, 0x2a, OPACITY));
        g.fillRect(
                round(WHEEL_PADDING_X * ROBOT_IMAGE_WIDTH), round(WHEEL_PADDING_Y * ROBOT_IMAGE_HEIGHT),
                round(WHEEL_WIDTH * ROBOT_IMAGE_WIDTH), round(WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT));
        g.fillRect(
                round(ROBOT_IMAGE_WIDTH - WHEEL_WIDTH * ROBOT_IMAGE_WIDTH - WHEEL_PADDING_X * ROBOT_IMAGE_WIDTH),
                round(WHEEL_PADDING_Y * ROBOT_IMAGE_HEIGHT), round(WHEEL_WIDTH * ROBOT_IMAGE_WIDTH),
                round(WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT));
        g.fillRect(
                round(ROBOT_IMAGE_WIDTH - WHEEL_WIDTH * ROBOT_IMAGE_WIDTH - WHEEL_PADDING_X * ROBOT_IMAGE_WIDTH),
                round(ROBOT_IMAGE_HEIGHT - WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT - WHEEL_PADDING_Y * ROBOT_IMAGE_HEIGHT),
                round(WHEEL_WIDTH * ROBOT_IMAGE_WIDTH), round(WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT));
        g.fillRect(
                round(WHEEL_PADDING_X * ROBOT_IMAGE_WIDTH),
                round(ROBOT_IMAGE_HEIGHT - WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT - WHEEL_PADDING_Y * ROBOT_IMAGE_HEIGHT),
                round(WHEEL_WIDTH * ROBOT_IMAGE_WIDTH), round(WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT));

        // Draw the direction indicator:
        g.setColor(new Color(0x74, 0x2a, 0x2a));
        g.fillRect(round(ROBOT_IMAGE_WIDTH / 2.0 - DIRECTION_LINE_WIDTH * ROBOT_IMAGE_WIDTH / 2.0), 0,
                round(DIRECTION_LINE_WIDTH * ROBOT_IMAGE_WIDTH),
                round(ROBOT_IMAGE_HEIGHT * DIRECTION_LINE_HEIGHT));
    }

    // Render just the robot:
    void renderRobot(Graphics2D g) {
        Pose2d simulationPose = simulation.getPose(0);
        AffineTransform imageTransform = new AffineTransform();
        imageTransform.translate(simulationPose.position.x, simulationPose.position.y);
        imageTransform.scale(1.0 / ROBOT_IMAGE_WIDTH,1.0 / ROBOT_IMAGE_HEIGHT);
        imageTransform.rotate(simulationPose.heading.log() + Math.toRadians(90));
        imageTransform.scale(WilyCore.config.robotWidth, WilyCore.config.robotLength);
        imageTransform.translate(-ROBOT_IMAGE_HEIGHT / 2.0, -ROBOT_IMAGE_HEIGHT / 2.0);
        setAlpha(g, 0.5);
        g.drawImage(robotImage, imageTransform, null);
        setAlpha(g, 1.0);
    }

    // Set the transform to use inches and have the origin at the center of field. This
    // returns the current transform to restore via Graphics2D.setTransform() once done:
    public static AffineTransform setFieldTransform(Graphics2D g) {
        // Prime the viewport/transform and the clipping for field and overlay rendering:
        AffineTransform oldTransform = g.getTransform();
        g.setClip(FIELD_VIEW.x, FIELD_VIEW.y, FIELD_VIEW.width, FIELD_VIEW.height);
        g.transform(new AffineTransform(
                FIELD_SURFACE_DIMENSION / 144.0, 0,
                0, -FIELD_SURFACE_DIMENSION / 144.0,
                FIELD_SURFACE_DIMENSION / 2.0 + FIELD_INSET,
                FIELD_SURFACE_DIMENSION / 2.0 + FIELD_INSET));
        return oldTransform;
    }

    // Set the page frame transform, returning the old transform:
    public static AffineTransform setPageFrameTransform(Graphics2D g) {
        AffineTransform oldTransform = g.getTransform();
        g.setTransform(new AffineTransform(
                FIELD_SURFACE_DIMENSION / 144.0, 0,
                0, FIELD_SURFACE_DIMENSION / 144.0,
                FIELD_INSET, FIELD_INSET));
        return oldTransform;
    }

    // Render the field, the robot, and the field overlay:
    public void render(Graphics2D g) {
        // Lay down the background image without needing a transform:
        g.drawImage(backgroundImage, FIELD_VIEW.x + FIELD_INSET, FIELD_VIEW.y + FIELD_INSET, null);

        AffineTransform oldTransform = setFieldTransform(g);
        if (FtcDashboard.fieldOverlay != null)
            FtcDashboard.fieldOverlay.render(g);
        renderRobot(g);
        g.setTransform(oldTransform);
    }

    // Set the global alpha in the range [0.0, 1.0]:
    void setAlpha(Graphics2D g, double alpha) {
        g.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, (float) alpha));
    }

    // Render a field-of-view polygon:
    void renderFieldOfView(Graphics2D g, Point origin, double orientation, double fov) {
        double LENGTH = 48;
        Point p1 = new Point(LENGTH, 0).rotate(orientation + fov / 2).add(origin);
        Point p2 = new Point(LENGTH, 0).rotate(orientation - fov / 2).add(origin);

        // Create a polygon:
        GeneralPath path = new GeneralPath();
        path.moveTo(origin.x, origin.y);
        path.lineTo(p1.x, p1.y);
        path.lineTo(p2.x, p2.y);

        // Draw the polygon:
        setAlpha(g, 0.3);
        g.fill(path);
        setAlpha(g, 1.0);
    }

    // For the start screen, render an overlay over the initial field image:
    public void renderStartScreenOverlay(Graphics2D g) {
        g.drawImage(compassImage, 20, 20, null);

        AffineTransform oldTransform = setFieldTransform(g);
        for (WilyWorks.Config.Camera camera: WilyCore.config.cameras) {
            g.setColor(new Color(0xffffff));
            renderFieldOfView(g, new Point(camera.x, camera.y), camera.orientation, camera.fieldOfView);
        }
        g.setTransform(oldTransform);
    }
}
