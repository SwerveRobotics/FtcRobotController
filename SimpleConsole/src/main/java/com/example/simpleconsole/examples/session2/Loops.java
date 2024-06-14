package com.example.simpleconsole.examples.session2;

import com.example.simpleconsole.userinterface.Scanner;
import com.example.simpleconsole.userinterface.Sleeper;

// Demonstration of for, while, and do-while loops.
public class Loops {
    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        // i is often used for counting in loops.
        for (int i = 1; // First set an integer i equal to 1.
             i <= 5; // Run this code while i is less than or equal to 5.
             i = i + 1) { // Increase i by 1 at the end every loop.
            System.out.println(i);
        }
        // Use the for loop when you know (or have a variable for) how many times you
        //      want to repeat the code.

        System.out.println();

        System.out.print("Tell me the amount of time to wait in seconds: ");
        double targetTime = Double.parseDouble(scanner.nextLine()) * 1000;

        // Takes time as a number of milliseconds since the 1 January 1970.
        double startTime = System.currentTimeMillis();
        double elapsedTime = 0.0;

        while (elapsedTime // While the elapsed time
                < targetTime) { // Is less than the target time:
            elapsedTime = (System.currentTimeMillis()) - startTime;
            System.out.println(elapsedTime / 1000); // Print the elapsed time in seconds
            Sleeper.sleep(100); // Waits for 100 milliseconds
        }

        // Use the while loop when you want to repeat code until a condition changes, but the number
        //     of repetitions isn't fixed. Note that a while loop does need to run even a single
        //     iteration if the condition is false at the beginning.

        String utterance;
        do {
            System.out.print("Say whatever you want. Say \"quit\" to quit: ");
            utterance = scanner.nextLine();
            if (utterance.equals("quit")) {
                System.out.println("You asked to quit.");
            } else {
                System.out.println("You said: \"" + utterance + "\"");
            }
        } while (!utterance.equals("quit")); // While utterance does not equal "quit, continue loop

        // Use the do-while loop when you need to run the code at least once
        //     before checking the condition.

        System.out.println();

        // How to achieve the same functionality with all three types of loops:
        for (int i = 5; i > 0; i = i - 1) {
            System.out.println(i);
        }

        System.out.println();

        int i = 5;
        while (i > 0) {
            System.out.println(i);
            i = i - 1;
        }

        System.out.println();

        i = 5;
        do {
            System.out.println(i);
            i = i - 1;
        } while (i > 0);

        // Break can be used in both switch statements and loops:
        i = 5;
        while (true) {
            System.out.println(i);
            i = i - 1;
            if (i <= 0) {
                break;
            }
        }
    }
}
