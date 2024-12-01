// Class for simulating the arm and drawing it to the field view of FTC Dashboard.
static class ArmSimulator {
    final double ARM_LENGTH = 18; // Arm length, in inches
    final Vector2d ARM_BASE_OFFSET = new Vector2d(8, 0); // Base of arm on robot, relative to
    //   center of rotation
    final double END_ANGLE_IN_DEGREES = 185; // Arm rotates from 0° to 185°
    final double END_ANGLE_IN_TICKS = 2000; // It takes 2000 ticks to go from 0° to 185°
    final double VELOCITY = 3000; // The arm rotates at about 2000 ticks per second

    double currentArmPosition = 0; // Current arm motor position, in ticks
    double targetArmPosition = 0; // Target arm motor position, in ticks
    double intakePower = 0; // Current intake servo power, positive is intake, negative is deposit
    double previousTime; // Seconds

    // Return the time, in seconds:
    double time() {
        return nanoTime() * 1e-9;
    }

    // These simulate the 'real' helper functions:
    void setArmPosition(int targetInTicks) {
        this.targetArmPosition = targetInTicks;
        previousTime = time();
    }
    int getArmPosition() {
        return (int) currentArmPosition;
    }
    void setIntakePower(double power) {
        intakePower = power;
    }

    // This method updates our simulation and draws the arm's state on FTC dashboard.
    void update(Canvas canvas, Pose2d pose) {
        // Figure out the change in time from the last update call, in seconds:
        double currentTime = time();
        double dt = currentTime - previousTime;
        previousTime = currentTime;

        // Advance the arm's position according to our simple simulation:
        double remainingTicks = targetArmPosition - currentArmPosition;
        double magnitudeToAdvance = Math.min(VELOCITY * dt, Math.abs(remainingTicks));
        currentArmPosition += Math.copySign(magnitudeToAdvance, remainingTicks);

        // Draw a side view of the arm in the middle of the FTC Dashboard field, on top of
        // the submersible floor. We use the same coordinates for this as when we're
        // calculating the coordinates for the robot, so here the submersible base where
        // we'll be drawing goes from (-12, 22) to (12, -22), in inches:
        canvas.setFill("#808080"); // Set the fill color to grey
        canvas.fillRect(-12, 22, 24, -44); // Erase the submersible base

        canvas.setStroke("#000000"); // Draw the robot base in black
        canvas.strokeLine(0, 0, 18, 0); // The line representing the robot base

        canvas.setFill("#000000"); // Draw the wheels in black
        canvas.fillCircle(3, 0, 2); // Wheel #1
        canvas.fillCircle(15, 0, 2); // Wheel #2

        // Draw a line representing the arm. It starts at the origin and has an angle that
        // is calculated as a fraction of the end angles:
        double armAngle = Math.toRadians(END_ANGLE_IN_DEGREES * currentArmPosition / END_ANGLE_IN_TICKS);
        double xArmEnd = Math.cos(armAngle) * ARM_LENGTH;
        double yArmEnd = Math.sin(armAngle) * ARM_LENGTH;
        canvas.strokeLine(0, 0, xArmEnd, yArmEnd);

        // Draw an overhead view of the arm, on top of the robot pose:
        double robotAngle = pose.heading.toDouble();

        // The arm extends straightforward from its base; the arm's extent is the cosine of the
        // arm's current angle. Calculate the offset in robot-relative coordinates to the
        // intake at the end of the arm:
        Vector2d intakeOffset = ARM_BASE_OFFSET.plus(new Vector2d(-Math.cos(armAngle) * ARM_LENGTH, 0));

        // Rotate the offsets by the robot's current heading, and then add to the robot's
        // current pose position, to determine the overhead location of the base of the arm
        // and the intake, in field coordinates:
        Vector2d armBasePoint = pose.position.plus(MecanumDrive.rotateVector(ARM_BASE_OFFSET, robotAngle));
        Vector2d intakePoint = pose.position.plus(MecanumDrive.rotateVector(intakeOffset, robotAngle));

        // Draw a line from the arm's base point to the intake point:
        canvas.strokeLine(armBasePoint.x, armBasePoint.y, intakePoint.x, intakePoint.y);

        // Draw a circle at the end of the arm to represent the state of the intake motor.
        // Green is in-taking, red is out-taking, black is off. Draw it for both the side
        // view and the overhead view:
        if (intakePower > 0)
            canvas.setFill("#00ff00"); // Green
        else if (intakePower < 0)
            canvas.setFill("#ff0000"); // Red
        else
            canvas.setFill("#000000"); // Black

        canvas.fillCircle(intakePoint.x, intakePoint.y, 2);
        canvas.fillCircle(xArmEnd, yArmEnd, 2);
    }
}