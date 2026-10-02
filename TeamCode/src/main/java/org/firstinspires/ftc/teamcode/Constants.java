//package org.firstinspires.ftc.teamcode;
//
//import com.pedropathing.follower.Follower;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class Constants {
//    public static Follower create(HardwareMap h) {
//        // return new Follower(Drivetrain, Localizer, Foresight);
//        return null;
//    }
//}

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
        c.frontLeftName.set("FrontLeft");
        c.frontRightName.set("FrontRight");
        c.backLeftName.set("BackLeft");
        c.backRightName.set("BackRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
        });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-7.43202089324711);
        c.yPodOffset.set(1.4733589352585201);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.19284688978173262);
                Controller secondaryTranslationalForward = Controller.proportional(0.07125177281055176);
                Controller primaryTranslationalLateral = Controller.proportional(0.22259139943528283);
                Controller secondaryTranslationalLateral = Controller.proportional(0.08224157433960277);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017261379401325657));
                c.brake.set(Controller.proportionalFeedforward(0.014672172491126808));

                c.headingFeedback.set(Controller.proportional(3.896126450807905));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04621992643100665, 0.005968279276102467));

                c.linearBrakeCoefficients.set(Matrix.diag(0.02720910865782083, 0.07001028054640382));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0026267286388200486, 0.0011015415200308796));

                c.maxAchievableForwardVelocity.set(57.87780724973228);
                c.maxAchievableStrafeVelocity.set(45.608974029737105);
                c.naturalForwardDeceleration.set(33.81510833580937);
                c.naturalStrafeDeceleration.set(52.368071659730205);
            }
    );

    public static Follower create(HardwareMap h) {


        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );

    }
}