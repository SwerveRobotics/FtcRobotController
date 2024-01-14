package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

/**
 * Wrapper class for MeepMeep testing. Modify the 'myBot' constructor settings to reflect your
 * own robot. You shouldn't need to modify any other code here.
 */
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new RoadRunnerBotEntity(
                meepMeep,
                // Robot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width:
                new Constraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15),
                // Robot dimensions: width, height:
                15, 15,
                new Pose2d(0, 0, 0),
                meepMeep.getColorManager().getTheme(),
                0.8f,
                DriveTrainType.MECANUM,
                false);

        MecanumDrive drive = new MecanumDrive(myBot.getDrive());
        boolean distanceTest = false;
        if (distanceTest) {
            PropDistanceFactory prop = new PropDistanceFactory(drive);
            myBot.runAction(prop.getMeepMeepAction());

        } else {
            AutonDriveFactory auton = new AutonDriveFactory(drive);
            myBot.runAction(auton.getMeepMeepAction());
        }

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

/**
 * Shim so that the AutonDriveFactory can refer to the drive using a MecanumDrive type both
 * here in MeepMeep and also in the competition code.
 */
class MecanumDrive {
    DriveShim shim;
    MecanumDrive(DriveShim shim) {
        this.shim = shim;
    }
    TrajectoryActionBuilder actionBuilder(Pose2d pose) {
        return this.shim.actionBuilder(pose);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * This is a template for your autonomous code. It's called a "factory" because it creates objects
 * (Road Runner 'Action' objects in this case). Feel free to rename it to whatever you
 * prefer. By making this class independent of all of your other classes, and independent of any
 * FTC code, it can let you ^C copy this entire class back and forth between your competition
 * code and this MeepMeep test code. But if you add a dependency on something like DcMotor (which
 * is FTC code), it will stop compiling in MeepMeep and you won't be able to test your logic
 * anymore.
 */
class AutonDriveFactory {
    MecanumDrive drive;
    double xOffset;
    double yMultiplier;

    double parkingOffset;

    double parkingOffsetCenterFar;

    double centerMultiplier;

    double centerOffset;
    AutonDriveFactory(MecanumDrive drive) {
        this.drive = drive;
    }

    /*
     * Call this routine from your robot's competition code to get the sequence to drive. You
     * can invoke it there by calling "Actions.runBlocking(driveAction);".
     */
    enum SpikeMarks {
        LEFT,
        CENTER,
        RIGHT
    }

    class PoseAndAction {
        Action action;
        Pose2d startPose;

        PoseAndAction(Action action, Pose2d startPose) {
            this.action = action;
            this.startPose = startPose;
        }
    }

    /* Booleans 'isRed' (red or blue side), 'isFar' (far or close to backdrop)
     'location' (center, middle, or right), and 'intake' (Action for use).
     */
    PoseAndAction getDriveAction(boolean isRed, boolean isFar, SpikeMarks location, Action intake) {

         if (isFar) {
            xOffset = 0;
            parkingOffset = 55;
            centerMultiplier = 1;
            centerOffset = 0;
            if (location == xForm(SpikeMarks.CENTER)) {
                parkingOffset = 100;
            }
        } else {
            xOffset = 48;
            parkingOffset = 2;
            centerMultiplier = -1;
            centerOffset = 96;
        }

        if (isRed) {
            yMultiplier = 1;
        } else {
            yMultiplier = -1;
        }
        // in MeepMeep, intake needs to be null however .stopAndAdd() can't be null because it will crash so we set to a random sleep
        if(intake == null) {
            intake = new SleepAction(3);
        }

        TrajectoryActionBuilder spikeLeft = this.drive.actionBuilder(xForm(new Pose2d(-34, -64, Math.toRadians(90))));
        spikeLeft = spikeLeft.splineTo(xForm(new Vector2d(-34, -37)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-35, -34)), xForm((Math.toRadians(180))))
                //.stopAndAdd(intake)
                .splineToConstantHeading(xForm(new Vector2d(-30, -34)), xForm(Math.toRadians(180)))
                .splineTo(xForm(new Vector2d(-34, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(24 - xOffset , -12)), xForm(Math.toRadians(0)))
                .turn(Math.toRadians(180)) //Turn so the arm faces the backdrop
                .setTangent(xForm(Math.toRadians(0)))
                //.afterTime(0, moveDumper)
                .splineToConstantHeading(xForm(new Vector2d(48 - xOffset, -29.5)), xForm(Math.toRadians(0)));
                //.stopAndAdd(moveArm);

        TrajectoryActionBuilder spikeCenter = this.drive.actionBuilder(xForm(new Pose2d(-34, -64, (Math.toRadians(90)))));
        spikeCenter = spikeCenter.splineTo(xForm(new Vector2d(-34, -37)), xForm(Math.toRadians(90)))
                //.stopAndAdd(intake)
                .setTangent(xForm(Math.toRadians(-90)))
                .splineTo(xForm(new Vector2d(-34, -39)), xForm(Math.toRadians(-90)))
                .setTangent(xForm(Math.toRadians(180)))
                .splineTo(xForm(new Vector2d(-55, -39)), xForm(Math.toRadians(180)))
                .setTangent(xForm(Math.toRadians(90)))
                //.splineTo(xForm(new Vector2d(-55, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(24, -12)), xForm(Math.toRadians(0)))
                .turn(Math.toRadians(180)) //Turn so the arm faces the backdrop
                .setTangent(xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(48 - xOffset, -36)), xForm(Math.toRadians(0)));

        TrajectoryActionBuilder spikeRight = this.drive.actionBuilder(xForm(new Pose2d(-34, -64, Math.toRadians(90))));
        spikeRight = spikeRight.splineTo(xForm(new Vector2d(-35, -37)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-33, -37)), xForm(Math.toRadians(0)))
                .stopAndAdd(intake)
                .splineToConstantHeading(xForm(new Vector2d(-40, -34)), xForm(Math.toRadians(0)))
                .splineTo(xForm(new Vector2d(-36, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(24, -12)), xForm(Math.toRadians(0)))
                .turn(Math.toRadians(180)) //Turn so the arm faces the backdrop
                .setTangent(xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(48 - xOffset, -44)), xForm(Math.toRadians(0)));

        if(location == SpikeMarks.LEFT) {
            return new PoseAndAction(spikeLeft.build(), xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        } else if(location == SpikeMarks.CENTER) {
            return new PoseAndAction(spikeCenter.build(), xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        } else {
            return new PoseAndAction(spikeRight.build(), xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        }
    }

                // arm action
                /*.splineToConstantHeading(xForm(new Vector2d(-40, -34)), xForm(Math.toRadians(0)))
                .splineTo(xForm(new Vector2d(-36, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(58, -10)), xForm(Math.toRadians(0)));*/





    Pose2d xForm(Pose2d pose) {
        return new Pose2d(pose.position.x + xOffset, pose.position.y * yMultiplier, pose.heading.log() * yMultiplier);
    }

    Pose2d xFormCenter(Pose2d pose) {
        return new Pose2d((pose.position.x + centerOffset), pose.position.y * yMultiplier, pose.heading.log() * yMultiplier);
    }

    Vector2d xForm(Vector2d vector) {
        return new Vector2d(vector.x + xOffset, vector.y * yMultiplier);
    }

    double xForm(double angle) {
        return (angle * yMultiplier);
    }

    Vector2d xFormCenter(Vector2d vector) {
        return new Vector2d((vector.x + centerOffset), vector.y * yMultiplier);
    }

    SpikeMarks xForm(SpikeMarks spike) {
        if (yMultiplier == -1) {
            switch (spike) {
                case LEFT:
                    return SpikeMarks.RIGHT;
                case RIGHT:
                    return SpikeMarks.LEFT;
            }
        }
        return spike;
    }


    /*
     * MeepMeep calls this routine to get a trajectory sequence action to draw. Modify the
     * arguments here to test your different code paths.
     */
    Action getMeepMeepAction() {
        return getDriveAction(true, true, SpikeMarks.CENTER, null).action;
    }
}

class PropDistanceFactory {

    private final double xOffset = -24;
    private final double yOffset = 0;
    MecanumDrive drive;
    PropDistanceFactory(MecanumDrive drive) {
        this.drive = drive;
    }

    Pose2d xForm(double x, double y, double theta, boolean isRed, boolean isFar) {
        if (!isFar)
            x = x * -1 + xOffset;

        if (!isRed)
            y = y * -1 + yOffset;

        return new Pose2d(x, y, theta);
    }
    Action getDistanceAction(boolean isRed, boolean isFar, Action sweepAction) {
        double tangent = Math.PI / 2;

        if (sweepAction == null) {
            sweepAction = new SleepAction(0.5);
        }

        if (!isRed) {
            tangent = tangent * -1;
        }

        Pose2d startPose = xForm(-34, -60, Math.PI /2, isRed, isFar);

        TrajectoryActionBuilder builder
                = this.drive.actionBuilder(startPose)
                .setTangent(tangent)
                .splineToLinearHeading(xForm(-42, -45, Math.PI / 2, isRed, isFar), tangent)
                .afterTime(0, sweepAction)
                .turn(2 * Math.PI)
                .setTangent(-tangent)
                .splineToLinearHeading(startPose, -tangent);

        return builder.build();
    }
    Action getMeepMeepAction() {
        return getDistanceAction(false, false, null);
    }
}
