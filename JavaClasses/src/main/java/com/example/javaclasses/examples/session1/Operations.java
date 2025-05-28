package com.example.javaclasses.examples.session1;

/* Usually, programmers only comment on code that is unclear or needs special attention. If a line
of code is super long, like this one, line breaks are usually inserted to keep all the code to the
left of the grey line on the right. To help with line breaks in comments, multi-line comments were
invented! */

// This class takes input from the console and does something with it.
public class Operations {
    public static void main(String[] args) {
        // This initializes the variables xDouble as an double.
        double xDouble;
        xDouble = 4;
        // Or, just:
        double yDouble = 3;

        // Another way to initialize
        int xInt, yInt;

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

        // You can store results of calculations in variables too!
        int result;

        result = xInt + yInt;

        // Notice the difference specifically in integer division.
        System.out.println("Integer addition: " + result);
        System.out.println("Integer subtraction: " + (xInt - yInt));
        System.out.println("Integer multiplication: " + (xInt * yInt));
        System.out.println("Integer division: " + (xInt / yInt));

        // Prints a blank line
        System.out.println();

        // Parentheses are very important!
        System.out.println("Concatenation: " + xDouble + yDouble);
    }
}
