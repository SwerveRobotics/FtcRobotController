package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team417.roadrunner.KinematicType;

abstract public class BaseOpModeSlowBot extends LinearOpMode {

    //TODO: Tune correct value
    final static double X_WRIST_FOLDED_IN = 0;
    //TODO: tune for correct value
    final static double SLIDE_HOME_POSITION = 0;
    //TODO: tune for correct value
    final static double LIFT_REST_POSITION = 0;

    // This method checks if the linear slide is extended or not. Returns true if it is extended, false if not
    // TODO: implement this
    public boolean isSlideExtended(){
        return false;
    }

    // This helper method controls the linear slides and tells motor to go to desired position in ticks
    // TODO: implement this
    public void controlSlide(double positionInTicks){}

    // This method controls the 4bar to desired height
    // TODO: implement this
    public void moveLift(double heightInTicks){}

    // This method moves the wrist up and down based on desired position
    // TODO: implement this
    public void wristControl(double wristPosition){}

    // Controls the intake of the wrist
    // TODO: implement this
    public void intakeControl(double spinControl){}

    // TODO: implement this
    final static double INTAKE_DEPOSIT = 0.0;
    final static double INTAKE_COLLECT = 0.0;
    final static double INTAKE_OFF = 0.0;
    final static double LIFT_COLLECT = 0.0;
    final static double WRIST_OUT = 0.0;
    final static double WRIST_IN = 0.0;
    final static double LIFT_CLEAR_BARRIER = 0.0;
    // Slow bot variables
    double liftPosition = LIFT_REST_POSITION;
    double slidePosition = SLIDE_HOME_POSITION;
    final static double linearSlideHome = 0.0;
    final static double SLIDE_SCORE_IN_BASKET = 0;
    final static double LIFT_SCORE_SPECIMEN = 0.0;
    final static double LIFT_SCORE_LOW_BASKET = 0.0;

    // RC 17.50
    // DEV 17.75
    final static double ROBOT_LENGTH = 17.50;
    // RC 16.50
    // DEV 18.50
    final static double ROBOT_WIDTH = 16.50;

    public void initializeHardware() {}

    public static final KinematicType kinematicType = KinematicType.X;

}
