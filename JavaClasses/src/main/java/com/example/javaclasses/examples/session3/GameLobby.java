package com.example.javaclasses.examples.session3;

public class GameLobby {
    public static void main(String[] args) {
        // Greet Player 1
        greetPlayer("Astro_Alex", "Gold");

        // Greet Player 2
        greetPlayer("Cpde,alster", "Silver");

        // Greet Player 3
        greetPlayer("Pixel", "Bronze");
    }

    public static void greetPlayer(String name, String rank) {
        System.out.println("====================");
        System.out.println("Welcome, " + name + "!");
        System.out.println("Your rank is: "+ rank);
        System.out.println("Connecting you to the server...");
        System.out.println("====================");
    }
}
