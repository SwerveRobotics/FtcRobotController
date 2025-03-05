package com.example.javaclasses.examples.session1;

// This line imports the Scanner class, making it available for one to use.
import java.util.Scanner;

// This class takes input from the console and does something with it.
public class Operations {
    public static void main(String[] args) {
        // This creates a new Scanner object, allowing the program to take input from System.in, the
        //     standard input.
        Scanner scanner = new Scanner(System.in);

        // This initializes the variables xInt and yInt as integers.
        int xInt;
        int yInt;

        // Another way to initialize
        double xDouble, yDouble;

        // Or, you can assign a value to a variable when initializing it
        String prompt = "Enter a decimal number or integer: ";

        // System.out.print(String) prints a statement without adding a new line after it.
        System.out.print(prompt);

        // This method provides data from the input as a String. The result from the method is
        //     assigned to the variable xString.
        String xString = scanner.nextLine();

        // System.out.println(String) prints a statement while adding a new line after it.
        System.out.print(prompt);

        // This statements assigns the result to yString.
        String yString = scanner.nextLine();

        // This method turns the Strings into doubles. Then we assign the doubles to our xDouble and
        //     yDouble variables.
        xDouble = Double.parseDouble(xString);
        yDouble = Double.parseDouble(yString);

        xInt = Integer.parseInt(xString);
        yInt = Integer.parseInt(yString);

        // Here we turn the doubles into integers. Some precision is lost (anything after the
        //     decimal point).
        xInt = (int) xDouble;
        yInt = (int) yDouble;

        // Prints a blank line
        System.out.println();

        // Here are the variables:
        System.out.println("xDouble: " + xDouble);
        System.out.println("yDouble: " + yDouble);

        // Prints a blank line
        System.out.println();

        System.out.println("xInt: " + xInt);
        System.out.println("yInt: " + yInt);

        // Prints a blank line
        System.out.println();

        // Now we can print the results of mathematical calculations:
        System.out.println("Double addition: " + (xDouble + yDouble));
        System.out.println("Double subtraction: " + (xDouble - yDouble));
        System.out.println("Double multiplication: " + (xDouble * yDouble));
        System.out.println("Double division: " + (xDouble / yDouble));

        // Prints a blank line
        System.out.println();

        int result;

        result = xInt + yInt;

        // Notice the difference specifically in integer division. For example, try x = 4 and y = 3.
        System.out.println("Integer addition: " + result);
        System.out.println("Integer subtraction: " + (xInt - yInt));
        System.out.println("Integer multiplication: " + (xInt * yInt));
        System.out.println("Integer division: " + (xInt / yInt));

        // Prints a blank line
        System.out.println();

        // Parentheses are very important!
        System.out.println("Double concatenation: " + xDouble + yDouble);
    }
}
