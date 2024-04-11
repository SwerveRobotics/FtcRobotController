package com.wilyworks.common;

import static java.lang.ClassLoader.getSystemClassLoader;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

/**
 * FTC programs use this class to configure and interface with the Wily Works simulator.
 */
public class WilyWorks {

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Configuration

    // Name of the opMode to automatically start, if any:
    static public String autoOpMode;

    // The robot's dimensions, in inches:
    static public final int robotWidth = 24;
    static public final int robotLength = 32;

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Interaction

    // WilyLink class for communicating with the Wily Works simulator:
    static private Class wilyCore = getWilyCore();

    // Check this boolean to determine whether you're running on the real robot or in a simulation:
    static public boolean isSimulating = (wilyCore != null);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Implementation

    // Initialize the link to WilyWorks without worrying about messy exceptions:
    static Class initializeWilyLink() throws ClassNotFoundException, NoSuchMethodException, InvocationTargetException, IllegalAccessException {
        Class klass = getSystemClassLoader().loadClass("com.wilyworks.simulator.WilyCore");
        // @@@ klass.getMethod("initialize").invoke(null, autoOpMode, robotWidth, robotLength);
        return klass;
    }

    // Wrap WilyLink initialization with
    static Class getWilyCore() {
        try {
            return initializeWilyLink();
        } catch (ClassNotFoundException|NoSuchMethodException|InvocationTargetException|IllegalAccessException e) {
            return null;
        }
    }

    // The pose is always field-relative:
    static public class Pose {
        double x, y; // Inches
        double heading; // Radians

        public Pose(double x, double y, double heading) {
            this.x = x; this.y = y; this.heading = heading;
        }
    }

    // The velocity will either be robot-relative or field-relative, depending on the API used:
    static public class Velocity {
        double x, y; // Inches/s
        double angular; // Radians/s
        public Velocity(double x, double y, double angular) {
            this.x = x; this.y = y; this.angular = angular;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Road Runner control

    // Set the robot to a given pose and (optional) velocity in the simulation:
    static public boolean setPose(Pose2d pose, PoseVelocity2d velocity) {
        if (wilyCore != null) {
            try {
                Method setPose = wilyCore.getMethod("setPose",
                        double.class, double.class, double.class,
                        double.class, double.class, double.class);
                setPose.invoke(null,
                        pose.position.x, pose.position.y, pose.heading.log(),
                        velocity.linearVel.x, velocity.linearVel.y, velocity.angVel);
                return true; // ====>
            } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
                throw new RuntimeException(e);
            }
        }
        return false;
    }

    // Set the drive powers:
    static public boolean setDrivePowers(
            // Manual power, normalized voltage from -1 to 1, robot-relative coordinates, can be null:
            PoseVelocity2d stickVelocity,
            // Computed power, inches/s and radians/s, field-relative coordinates, can be null:
            PoseVelocity2d assistVelocity) {
        if (wilyCore != null) {
            try {
                Method setDrivePowers = wilyCore.getMethod("setDrivePowers",
                        PoseVelocity2d.class, PoseVelocity2d.class);
                setDrivePowers.invoke(null, stickVelocity, assistVelocity);
                return true; // ====>
            } catch (InvocationTargetException|IllegalAccessException|NoSuchMethodException e) {
                throw new RuntimeException(e);
            }
        }
        return false;
    }

    // Get the localizer position and velocity from the simulation:
    static public Twist2dDual<Time> localizerUpdate() {
        if (wilyCore != null) {
            try {
                Method getLocalization = wilyCore.getMethod("getLocalization");
                double[] localization = (double[]) getLocalization.invoke(null);
                return new Twist2dDual<Time>(new Vector2dDual<Time>(
                        new DualNum<Time>(new double[] { localization[0], localization[3] }),
                        new DualNum<Time>(new double[] { localization[1], localization[4] })),
                        new DualNum<Time>(new double[] { localization[2], localization[5] }));

            } catch (InvocationTargetException|IllegalAccessException|NoSuchMethodException e) {
                throw new RuntimeException(e);
            }
        }
        return null;
    }
}
