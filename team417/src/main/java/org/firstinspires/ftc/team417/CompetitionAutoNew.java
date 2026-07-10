package org.firstinspires.ftc.team417;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team417.apriltags.LimelightAprilTagDetector;
import org.firstinspires.ftc.team417.apriltags.Pattern;
import org.firstinspires.ftc.team417.javatextmenu.MenuFinishedButton;
import org.firstinspires.ftc.team417.javatextmenu.MenuHeader;
import org.firstinspires.ftc.team417.javatextmenu.MenuInput;
import org.firstinspires.ftc.team417.javatextmenu.MenuSlider;
import org.firstinspires.ftc.team417.javatextmenu.MenuSwitch;
import org.firstinspires.ftc.team417.javatextmenu.TextMenu;
import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.roadrunner.RobotAction;

@Autonomous(name="CompetitionAutoNew", group="Competition", preselectTeleOp="CompetitionTeleOpNew")

public class CompetitionAutoNew extends BaseOpMode {



        @Override
        public void runOpMode() {
            TextMenu menu = new TextMenu();
            MenuInput menuInput = new MenuInput(MenuInput.InputType.CONTROLLER);
            initializeHardware();


        }
    }



