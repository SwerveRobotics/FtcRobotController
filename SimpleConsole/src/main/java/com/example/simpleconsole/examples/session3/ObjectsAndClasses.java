package com.example.simpleconsole.examples.session3;

import com.example.simpleconsole.userinterface.Scanner;

// A class can be declared outside of another class in the same file.
// It cannot be public, though. If it were public, it needs to be in its own file.
class Fruit {
    // Classes can also have variables; variables don't have to be in a method.
    // These variables are called attributes.
    // The field of a class consists of all the attributes.
    String name = "Fruit";

    // Attributes can also have modifiers, like methods.
    // Like variables in a method, you don't have to assign a value right away.
    public int nutritionValue;

    boolean juiced = false;

    // A constructor is a method that creates or initializes an object of a class.
    // The constructor must be named the name of the class.
    // For example, if the class was called Tree, the constructor must be named public Tree(...
    // Usually constructors should be public.

    public Fruit(String name, int nutritionValue) {
        System.out.println("Creating a Fruit object called \"" + name + "\"!");
        this.name = name;
        this.nutritionValue = nutritionValue;
    }

    // All classes can have methods.
    void juice(int addedSugars) {
        if (!juiced) {
            // Once again, it's best to split lines that are too long.
            System.out.println("Juicing an " + name + " and adding "
                    + addedSugars + " grams of sugar!");
            juiced = true;
            // A shorter way of writing nutritionValue = nutritionValue - addedSugars.
            nutritionValue -= addedSugars;
        }
        // A void method doesn't require a return statement.
    }
}

// Demonstration of how to use different classes in the same file
public class ObjectsAndClasses {
    public static void main(String[] args) {
        // Creating an object:
        Fruit orange = new Fruit("orange", 8);

        System.out.println();

        System.out.println("Before juicing: ");

        // Printing the attribute of an object:
        // The syntax is object.attributeName
        System.out.println("Orange nutrition value: " + orange.nutritionValue);
        System.out.println("Is orange juiced? " + orange.juiced);

        System.out.println();

        Scanner scanner = new Scanner(System.in);

        System.out.print("How many grams of sugar to add: ");
        int sugar = Integer.parseInt(scanner.nextLine());

        scanner.close();

        System.out.println();

        // Calling a method on an object:
        // The syntax is object.methodName(parameter1, parameter2...)
        orange.juice(sugar);

        System.out.println();

        System.out.println("After juicing: ");

        System.out.println("Orange nutrition value: " + orange.nutritionValue);
        System.out.println("Is orange juiced? " + orange.juiced);

        System.out.println();
        System.out.println();

        // Objects are confusing in that they are passed by reference.
        // This is unlike primitive types, like integers and doubles, which are passed by value.
        // A good rule of thumb is, if the type isn't capitalized, it's a primitive type:
        // e.g. int, double, char

        // If we initialize two apples:
        Fruit redApple = new Fruit("apple", 10);
        Fruit greenApple = new Fruit("apple", 10);

        System.out.println();

        // Even if these apples have have the same attributes,
        // the apples are still different apples. The red apple is not the green apple.
        System.out.println("Apples are the same apple? " + (redApple == greenApple));

        System.out.println();

        // This is unlike how numbers work, though. Every number is the same number.
        System.out.println("Numbers are the same number? " + (5 == 5));

        System.out.println();

        // So if I juice the green apple...
        greenApple.juice(3);

        System.out.println();

        System.out.println("Green apple nutrition value: " + greenApple.nutritionValue);
        System.out.println("Is green apple juiced? " + greenApple.juiced);

        System.out.println();

        // ...the red apple isn't juiced.
        System.out.println("Red apple nutrition value: " + redApple.nutritionValue);
        System.out.println("Is red apple juiced? " + redApple.juiced);

        System.out.println();
        System.out.println();

        // The correct way to pass by reference:
        Fruit myApple = redApple;

        // Now, my apple is the red apple.
        System.out.println("Apples are the same apple? " + (redApple == myApple));

        System.out.println();

        // Interestingly, because my apple is the red apple, if I juice my apple...
        myApple.juice(5);

        System.out.println();

        System.out.println("My apple nutrition value: " + myApple.nutritionValue);
        System.out.println("Is my apple juiced? " + myApple.juiced);

        System.out.println();

        // ...I am juicing the red apple.
        // (Of course, this works the other way around as well.)

        System.out.println("Red apple nutrition value: " + redApple.nutritionValue);
        System.out.println("Is red apple juiced? " + redApple.juiced);
    }
}
