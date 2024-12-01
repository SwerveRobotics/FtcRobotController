/*// Action for controlling the arm in Auto trajectories.
class ArmAction extends RobotAction {
    final double EPSILON = 3; // Ticks

    int targetArmPosition;
    double targetIntakePower;

    public ArmAction(int armPosition, double intakePower) {
        this.targetArmPosition = armPosition;
        this.targetIntakePower = intakePower;
    }

    @Override
    public boolean run(double elapsedTime) {
        setArmPosition(targetArmPosition);
        setIntakePower(targetIntakePower);
        double armDistance = Math.abs(getArmPosition() - targetArmPosition);

        // Return 'true' to call again when not at target position yet:
        return (armDistance > EPSILON);
    }

    // Arm positions, in ticks:
    final int ARM_HOME = 0;
    final int ARM_COLLECT = 2000;
    final int ARM_SCORE_BASKET = 1500;

    final double INTAKE_COLLECT = 0.5;
    final double INTAKE_DEPOSIT = -1.0;
    final double INTAKE_OFF = 0.0;

    // Mechanism state:
    DcMotorEx armMotor;
    CRServo intakeCRServo;
    ArmSimulator armSimulator = new ArmSimulator();

    // Helper function for settings the arm position, in ticks:
    void setArmPosition(int targetInTicks) {
        if ((targetInTicks < ARM_HOME) || (targetInTicks > ARM_COLLECT)) {
            throw new IllegalArgumentException("Invalid setArmPosition() request.");
        }
        armMotor.setTargetPosition(targetInTicks);
        armSimulator.setArmPosition(targetInTicks);
    }
    // Helper function for querying the arm position, in ticks. Uses the simulator when running
    // under Wily Works.
    int getArmPosition() {
        if (WilyWorks.isSimulating)
            return armSimulator.getArmPosition();
        else
            return armMotor.getCurrentPosition();
    }
    // Helper function to set the power on the intake.
    void setIntakePower(double power) { // Positive is intake, negative is out-take, zero is stop
        if ((power > INTAKE_COLLECT) || (power < INTAKE_DEPOSIT)) {
            throw new IllegalArgumentException("Invalid setIntakePower() request.");
        }
        intakeCRServo.setPower(power);
        armSimulator.setIntakePower(power);
    }
}*/