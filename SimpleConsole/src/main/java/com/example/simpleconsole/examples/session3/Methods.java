package com.example.simpleconsole.examples.session3;

import com.example.simpleconsole.userinterface.Scanner;

// Demonstration of how to declare and call methods, with and without parameters.
public class Methods {
    public static void main(String[] arg) {
        // Call the printMessage(String) function:
        printMessage("The program is starting!");

        // Call the exponent(int, int) function inside of the printMessage(String) function:
        printMessage("2 ^ 3 = " + exponent(2, 3));

        // Methods are great for things that must be done multiple times.
        int x = inputInteger("Input an x: ");
        int y = inputInteger("Input a y: ");

        printMessage(x + " ^ " + y + " = " + exponent(x, y));

        printMessage("Do you like this example?");

        // Call the inputsYes() function from within an if-block:
        if (inputsYes()) {
            printMessage("Great!");
        } else {
            printMessage("That's too bad.");
        }

        System.out.println();
    }

    public // Let every class be able to use this method
    static // Let this method be used even if an object of this class is not created
    void // Let this method not return anything
    printMessage // Create a method called "printStart"
    (String message // That takes a string "message"
    ) {
        System.out.println("The message is: " + message);
        System.out.println();
        return; // Go back to wherever this method was called from
        //System.out.println("This line of code will never run...");
    }

    private // Let only this class be able to use this method
    static // Let this method be used even if an object of this class is not created
    int // Let this method return an integer
    exponent // Create a method called exponent
    (int x, // That takes an integer "x"
     int y // And an integer "y"
    ) // As parameters
    {
        System.out.println("Raising " + x + " to the " + y + "th power...");

        // Repeatedly multiply x y times:
        int result = 1;
        for (int i = 0; i < y; i++) {
            result = result * x;
        }
        return result; // Go to wherever this method was called from and send the value of result
    }

    // It's usually best to declare a method as public in FTC!
    public static int inputInteger(String prompt) {
        System.out.println("Obtaining an integer...");

        System.out.print("The prompt is: " + prompt);
        Scanner scanner = new Scanner(System.in);
        String response = scanner.nextLine();

        // It's best to close the scanner after using it. This saves a bit of system resources.
        scanner.close();

        return Integer.parseInt(response); // Go back and send the value of response cast to an int
    }

    // You can also choose not to add a modifier or any parameters.
    // The return type is required, though!
    static boolean inputsYes() {
        System.out.println("Obtaining input...");

        System.out.print("The prompt is: Yes (y) or no (n): ");
        Scanner scanner = new Scanner(System.in);
        String response = scanner.nextLine();

        scanner.close();

        // The method String.equals(String) itself returns a boolean.
        return response.equals("y");
    }
}
