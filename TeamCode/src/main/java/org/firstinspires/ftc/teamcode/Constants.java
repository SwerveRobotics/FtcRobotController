package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
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
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig));}
    //public static class
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(3.389764);
        c.yPodOffset.set(-2.468504);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.29242406019400724);
                Controller secondaryTranslationalForward = Controller.proportional(0.10804287652689003);
                Controller primaryTranslationalLateral = Controller.proportional(0.4211347524580207);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15559803810540485);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012545179288876947));
                c.brake.set(Controller.proportionalFeedforward(0.010663402395545404));

                c.headingFeedback.set(Controller.proportional(4.781346491741541));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.061682526938248425, 0.0031073359575046274));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10258356784461263, 0.07298712731132045));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011906535260917026, 0.0016997471788383093));

                c.maxAchievableForwardVelocity.set(60.83105466938613);
                c.maxAchievableStrafeVelocity.set(51.824154457664804);
                c.naturalForwardDeceleration.set(29.130222373447047);
                c.naturalStrafeDeceleration.set(65.42127397318905);
            }
    );
}