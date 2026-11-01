package org.firstinspires.ftc.teamcode.procedures;

import static com.pedropathing.ivy.commands.Commands.waitMs;

import static java.lang.reflect.Array.getDouble;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Constants;

public class CustomFunctions {
    public Command followTimeout(Follower follower, Path path, double millis) {
        return new CommandBuilder()
                .setStart(() -> follower.follow(path))
                .setDone(follower::atParametricEnd).raceWith(waitMs(millis));
    }
    public Command followTimeout(Follower follower, Path path) {
        double millis = (path.curve.length())/((Constants.foresightConfig.maxAchievableStrafeVelocity.get())) * 1.3;
        return new CommandBuilder()
                .setStart(() -> follower.follow(path))
                .setDone(follower::atParametricEnd).raceWith(waitMs(millis));
    }
}
