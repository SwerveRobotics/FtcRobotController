package org.firstinspires.ftc.robotcore.internal.ui;

/** A *typed* integer so that we can more easily keep track of things */
public enum GamepadUser
{
    ONE(1), TWO(2);

    public byte id;
    GamepadUser(int id) { this.id = (byte)id; }

    public static GamepadUser from(int user)
    {
        if (user==1) return ONE;
        if (user==2) return TWO;
        return null;
    }
}
