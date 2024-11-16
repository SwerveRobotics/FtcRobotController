package org.firstinspires.ftc.team417.programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team417.Config;
import org.firstinspires.ftc.team417.menu.MenuFinishedButton;
import org.firstinspires.ftc.team417.menu.MenuHeader;
import org.firstinspires.ftc.team417.menu.MenuInput;
import org.firstinspires.ftc.team417.menu.MenuSlider;
import org.firstinspires.ftc.team417.menu.MenuSwitch;
import org.firstinspires.ftc.team417.menu.TextMenu;

abstract public class BaseOpMode extends LinearOpMode {
    TextMenu menu = new TextMenu();

    public void runMenu() {
        MenuInput menuInput = new MenuInput(MenuInput.InputType.CONTROLLER);

        menu.add(new MenuHeader("You are running 417's Auto!"))
                .add("Shortcut to header?")
                .add()
                .add("Pick which robot:")
                .add("robot", Config.Robots.class)
                .add()
                .add("Use distance sensor?")
                .add("distance", new MenuSwitch(false))
                .add()
                .add("Use reliable auto?")
                .add("reliable", new MenuSwitch(false))
                .add()
                .add("Pick which alliance:")
                .add("alliance", Config.Alliances.class)
                .add()
                .add("Pick which location:")
                .add("location", Config.Locations.class)
                .add()
                .add("Pick the wait time:")
                .add("waitTime", new MenuSlider(Config.minWaitTime, Config.maxWaitTime, Config.scaleWaitTime))
                .add()
                .add("finishButton", new MenuFinishedButton());

        while (!menu.isCompleted()) {
            // get x,y (stick) and select (A) input from controller
            menuInput.update(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.a);
            menu.updateWithInput(menuInput);
            // display the updated menu
            for (String line : menu.toListOfStrings()) {
                telemetry.addLine(line); // but with appropriate printing method
            }
            telemetry.update();
        }
    }

    public void getResultsFromMenu() {
        Config.robot = menu.getResult(Config.Robots.class, "robot");
        Config.useDistance = menu.getResult(Boolean.class, "distance");
        Config.useReliableAuto = menu.getResult(Boolean.class, "reliable");
        Config.alliance = menu.getResult(Config.Alliances.class, "alliance");
        Config.location = menu.getResult(Config.Locations.class, "location");
        Config.waitTime = menu.getResult(Double.class, "waitTime");
        Config.used = true;
    }
}
