package com.example.simpleconsole;

import com.example.simpleconsole.ui.Scanner;

public class Main {
    // This is the main coding space for new members!
    public static void main(String[] args) {
        System.out.println("Hi!");
        System.out.println();
        System.out.println("Here are the numbers from one to ten: ");
        for (int i = 1; i < 11; i++) {
            System.out.println(i);
        }
        for (int i = 1; i < 11; i++) {
            System.out.print(i + " ");
        }
        Scanner scanner = new Scanner(System.in);

        System.out.println();
        System.out.print("Do you like it: ");
        String response = scanner.nextLine();

        scanner.close();

        System.out.println(String.format("You said, \"%s\"", response));
    }
}