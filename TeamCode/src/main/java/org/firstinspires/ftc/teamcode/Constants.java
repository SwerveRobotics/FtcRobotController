package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-2.9591904287263167);
        c.yPodOffset.set(-3.6325193390132875);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1834715822799642);
                Controller secondaryTranslationalForward = Controller.proportional(0.06778784720147853);
                Controller primaryTranslationalLateral = Controller.proportional(0.23180223674219166);
                Controller secondaryTranslationalLateral = Controller.proportional(0.08564473260639993);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012098025029805544));
                c.brake.set(Controller.proportionalFeedforward(0.010283321275334711));

                c.headingFeedback.set(Controller.proportional(3.682227369375198));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.043065916900860556, 0.004805709341740694));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06575273150704988, 0.051787563108627956));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001209285492968948, 0.00165762551241349));

                c.maxAchievableForwardVelocity.set(39.86544799977838);
                c.maxAchievableStrafeVelocity.set(36.35275429384472);
                c.naturalForwardDeceleration.set(17.975100345577395);
                c.naturalStrafeDeceleration.set(35.4132100583588);
            }
    );
}