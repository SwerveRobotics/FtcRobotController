package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "417 Auto")
public class CompetitionAuto extends BaseOpMode{
    //when you select the class
    private Follower follower;
    //the values of the paths
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(32.0169, 3.8886, 90);
    private final Pose path1 = poseFactory.of(31.6743, 28.8051, 0);
    private final Pose point2 = poseFactory.of(104, 38, -172.7547);
    private final ElapsedTime timer = new ElapsedTime();

    //what happens when you press the start button on the driver hub

    //telling the paths to go in order
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2())
        );
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //Making follower to tell the robot
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            //telling the values of the robot on the driver hub
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
    //Telling the robot that these paths are where to go
    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }
    public int zero() {
        return 0;
    }
    public Path path2() {
        return Paths.line(path1, point2).reverseTangent();
    } //6767676767
    public Command intakeBallForMillis(long millis) {
        Command intakeBall = Command.build()
                .setStart(() -> {
                    //something goes here
                    timer.reset();
                    intakeMotor.setPower(1);
                })
                .setDone(() ->
                        timer.milliseconds() > millis)
                //check if ball went in
                .setEnd(endCondition -> {
                    intakeMotor.setPower(0);
                    // executed on end
                })
                .requiring(intakeMotor);
        return  intakeBall;
    }
    public Command intakeBallForMillis(long millis) {
        Command intakeBall = Command.build()
                .setStart(() -> {
                    //something goes here
                    timer.reset();
                    intakeMotor.setPower(1);
                })
                .setDone(() ->
                        timer.milliseconds() > millis)
                //check if ball went in
                .setEnd(endCondition -> {
                    intakeMotor.setPower(0);
                    // executed on end
                })
                .requiring(intakeMotor);
        return  intakeBall;
    }
}
