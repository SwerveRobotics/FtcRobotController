package com.example.simpleconsole.userinterface;

public class Sleeper {
    public static void sleep(long l) {
        try {
            Thread.sleep(l);
        } catch (InterruptedException e) {

        }
    }
}
