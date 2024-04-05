package org.firstinspires.ftc.team417_CENTERSTAGE.apriltagsnew;

/*
    Stores all the info about AprilTags, including location on the field, size, ID, tilt, etc.
    Used in AprilTagDetectionConcept
*/

import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.AprilTagInfo;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class AprilTagInfoDump {
    // Sets the game database
    public static final AprilTagLibrary library = AprilTagGameDatabase.getCenterStageTagLibrary();

    // Stores the location of the AprilTags in an array
    public static final AprilTagInfo[] AprilTagInfoArray = new AprilTagInfo[] {
            // These measurements come from AprilTagGameDatabase
            new AprilTagInfo(1, library.lookupTag(1).tagsize, library.lookupTag(1).fieldPosition.getData()[0], library.lookupTag(1).fieldPosition.getData()[1], library.lookupTag(1).fieldPosition.getData()[2]),
            new AprilTagInfo(2, library.lookupTag(2).tagsize, library.lookupTag(2).fieldPosition.getData()[0], library.lookupTag(2).fieldPosition.getData()[1], library.lookupTag(2).fieldPosition.getData()[2]),
            new AprilTagInfo(3, library.lookupTag(3).tagsize, library.lookupTag(3).fieldPosition.getData()[0], library.lookupTag(3).fieldPosition.getData()[1], library.lookupTag(3).fieldPosition.getData()[2]),
            new AprilTagInfo(4, library.lookupTag(4).tagsize, library.lookupTag(4).fieldPosition.getData()[0], library.lookupTag(4).fieldPosition.getData()[1], library.lookupTag(4).fieldPosition.getData()[2]),
            new AprilTagInfo(5, library.lookupTag(5).tagsize, library.lookupTag(5).fieldPosition.getData()[0], library.lookupTag(5).fieldPosition.getData()[1], library.lookupTag(5).fieldPosition.getData()[2]),
            new AprilTagInfo(6, library.lookupTag(6).tagsize, library.lookupTag(6).fieldPosition.getData()[0], library.lookupTag(6).fieldPosition.getData()[1], library.lookupTag(6).fieldPosition.getData()[2]),
            new AprilTagInfo(7, library.lookupTag(7).tagsize, library.lookupTag(7).fieldPosition.getData()[0], library.lookupTag(7).fieldPosition.getData()[1], library.lookupTag(7).fieldPosition.getData()[2]),
            new AprilTagInfo(8, library.lookupTag(8).tagsize, library.lookupTag(8).fieldPosition.getData()[0], library.lookupTag(8).fieldPosition.getData()[1], library.lookupTag(8).fieldPosition.getData()[2]),
            new AprilTagInfo(9, library.lookupTag(9).tagsize, library.lookupTag(9).fieldPosition.getData()[0], library.lookupTag(9).fieldPosition.getData()[1], library.lookupTag(9).fieldPosition.getData()[2]),
            new AprilTagInfo(10, library.lookupTag(10).tagsize, library.lookupTag(10).fieldPosition.getData()[0], library.lookupTag(10).fieldPosition.getData()[1], library.lookupTag(10).fieldPosition.getData()[2]),
    };

    // Finds the april tag with a certain ID
    public static AprilTagInfo findTagWithId(int id) {
        for (AprilTagInfo aprilTagInfo : AprilTagInfoArray) {
            if (aprilTagInfo.id == id) {
                return aprilTagInfo;
            }
        }
        return null;
    }
}
