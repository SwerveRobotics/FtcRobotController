package com.example.javaclasses.examples.session2;

/* Usually, programmers only comment on code that is unclear or needs special attention. If a line
of code is super long, like this one, line breaks are usually inserted to keep all the code to the
left of the grey line on the right. To help with line breaks in comments, multi-line comments were
invented! */

import java.util.Scanner;

// Demonstration of if, else, and else-if.
public class Conditionals {
    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        System.out.print("Enter a number: ");
        int number = Integer.parseInt(scanner.nextLine());

        if (number > 10) { // If the number entered is greater than 10
            System.out.println("That's a big number!"); // Execute this statement
        } else if (number == 9) { // Otherwise, if it's equal to 9
            System.out.println("9 is my favorite number!"); // Execute this one
        } else { // Otherwise
            System.out.println("That's a boring, small number."); // Execute this one.
        }

        System.out.println();

        // The expression "number > 10" is a boolean value.
        boolean big = number > 10;
        System.out.println("The number is big: " + big);

        // Other booleans include:
        boolean mathIsBroken = 2 + 2 == 5;
        System.out.println("Math is broken: " + mathIsBroken);
        boolean mathWorks = 2 + 2 != 5;
        System.out.println("Math works: " + mathWorks);

        System.out.println();

        // == and != do not work for strings. Use String.equals(String) instead of ==.
        // Note that Java is case-sensitive!
        System.out.print("Enter your name: ");
        String name = scanner.nextLine();
        boolean nameIsJeremy = name.equals("Jeremy");
        System.out.println("You are Jeremy: " + nameIsJeremy);

        System.out.println();

        // !, &&, and || are boolean operators. They basically mean "not", "and", and "or".
        boolean jeremyAndMathWorks = nameIsJeremy && mathWorks;
        System.out.println("You're Jeremy and math works: " + jeremyAndMathWorks);
        boolean notJeremy = !nameIsJeremy;
        System.out.println("You are not Jeremy: " + notJeremy);
        boolean jeremyOrMathBroken = nameIsJeremy || mathIsBroken;
        System.out.println("Either you're Jeremy or math is broken or both: " + jeremyOrMathBroken);

        System.out.println();

        // You can combine boolean operators. Pay attention to parentheses!
        boolean complexCondition = (nameIsJeremy || mathIsBroken) && !big;
        System.out.println("Either you're Jeremy or math is broken or both,");
        System.out.println("and in addition, the number is not big: " + complexCondition);

        System.out.println();

        System.out.print("Enter a fruit (apple, banana, cherry): ");
        String fruit = scanner.nextLine();

        // If "fruit"
        switch (fruit) {
            case "apple": // Is equal to "apple":
                System.out.println("You chose apple. An apple a day keeps the doctor away!");
                break; // Stop and don't go through the rest of the statements
            case "banana": // Is equal to "banana":
                System.out.println("You chose banana. Bananas are high in potassium!");
                break; // Stop and don't go through the rest of the statements
            case "cherry": // Is equal to "cherry":
                System.out.println("You chose cherry. Cherries are delicious!");
                break; // Stop and don't go through the rest of the statements
            default: // Or if it's equal to none of these:
                System.out.println("I only know about apples, bananas, and cherries.");
        }

        System.out.println();

        // What happens if you don't use break at the end of every case?
        // (Because Java sees a quotation mark as ending a string, put a slash in front
        //     to "escape" it. Ask the instructor for more details if interested.)
        System.out.print("Enter a letter grade (\"A\", \"B\", or \"C\"): ");
        String grade = scanner.nextLine();

        switch (grade) {
            case "A":
                System.out.println("Excellent!");
            case "B":
                System.out.println("Good!");
            case "C":
                System.out.println("Fair!");
            default:
                System.out.println("Invalid grade.");
        }
    }
}