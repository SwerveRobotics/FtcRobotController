package org.firstinspires.ftc.team417.menu;/*
 * THIS IS A TESTING CLASS USED IN DEVELOPING TEXTMENU!
 * it's not necessary to copy this class over when using the project lol
 */

import java.util.*;

public class Main {

    static Scanner userIn = new Scanner(System.in);

    public static final Map<String, Integer> inputMap;

    static {
        inputMap = new HashMap<>();
        inputMap.put("w", 1);
        inputMap.put("s", -1);
        inputMap.put("a", -1);
        inputMap.put("d", 1);
    }

    public enum option1 {
        RED,
        BLUE,
        GREEN,
        YELLOW,
    }
    public enum option2 {
        ONE,
        TWO,
        THREE,
    }
    public enum option3 {
        HELLO_WORLD,
        GOODBYE,
        SUSPICIOUS,
        AMONGUS,
    }

    public static void main(String[] args) {

        String input = "";

        TextMenu menu = new TextMenu();
        menu.add("(Sample text for estimating the horizontal space)")
                .add("The Robot Controller app is obsolete. You should")
                .add("install the new version of this FTC season.")
                .add("To ensure correct operation of the IMU in this")
                .add()
                .add("colors")
                .add("op1", option1.class)
                .add()
                .add("numbers!! YUH")
                .add("op2", option2.class)
                .add()
                .add("Stuff")
                .add("op3", option3.class)
                .add()
                .add("other elements:")
                .add()
                .add("sl1", new MenuSlider(5.0, 10.0, 4.0))
                .add()
                .add("swi1", new MenuSwitch(true))
                .add()
                .add("fin1", new MenuFinishedButton())
        ;
        /*for (int i = 0; i < 10; i++) {
            menu.add("#"+i, new MenuSwitch(true, ""+i, "z"));
        }
        menu.add("fin1", new MenuFinishedButton());*/

        MenuInput menuInput = new MenuInput();

        clear();
        while (!menu.isCompleted()) {

            for (String line : menu.toListOfStrings()) {
                System.out.println(line);
            }
            input = userIn.nextLine().toLowerCase();
            clear();

            if (input.matches("quit|q")) {
                System.out.println("force quitting...\n");
                break;
            } else if (input.matches("[wasdc]")) {
                int x = input.matches("[ad]") ? inputMap.get(input) : 0;
                int y = input.matches("[ws]") ? inputMap.get(input) : 0;
                boolean select = input.matches("c");

                menuInput.update(x, y, select);
                menu.updateWithInput(menuInput);

            }
        }

        System.out.println(menu.getResult(option1.class, "op1"));
        System.out.println(menu.getResult(option2.class, "op2"));
        System.out.println(menu.getResult(option3.class, "op3"));
        System.out.println(menu.getResult(Double.class, "sl1"));
        System.out.println(menu.getResult(Boolean.class, "swi1"));
        System.out.println();

        switch (menu.getResult(option1.class, "op1")) {
            case RED:System.out.println("red!");break;
            case BLUE:System.out.println("blue!");break;
            case GREEN:System.out.println("green!");break;
            case YELLOW:System.out.println("yellow!");break;
        }
    }

    // yoinked from random stack overflow thread
    public static void clear() {
        System.out.print("\033[H\033[2J");
        System.out.flush();
    }
}