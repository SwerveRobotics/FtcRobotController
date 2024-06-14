package com.example.simpleconsole.userinterface;

import static com.example.simpleconsole.userinterface.Robotics.gamepad;

import java.awt.Canvas;
import java.awt.Font;
import java.awt.Graphics;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;

/**
 * This class implements a lightweight emulation of FTC Telemetry that can run on the PC.
 */
public class Telemetry extends OutputStream {
    WindowFrame windowFrame;
    Canvas canvas;
    ArrayList<String> lineList = new ArrayList<>();
    int line = 0;
    int horizontalDisplacement = 0;

    public Telemetry() {
        windowFrame = new WindowFrame("SimpleConsole", 500);
        windowFrame.setVisible(true);
    }

    public void addLine(String lineCaption) {
        lineList.add(lineCaption);
    }

    public void addLine() {
        lineList.add("");
    }

    public void addData(String caption, Object value) {
        lineList.add(String.format("%s: %s", caption, value.toString()));
    }

    // Writes to System.out
    @Override
    public void write(int i) throws IOException {
        int size = lineList.size();
        if (size == 0) {
            lineList.add("");
            size++;
        }
        String lastLine = lineList.get(size - 1);
        switch (i) {
            case 65535:
                break;
            case 8:
                lineList.set(size - 1, lastLine.substring(0, lastLine.length() - 1));
                break;
            case 10:
                lineList.add("");
                break;
            default:
                lineList.set(size - 1, lastLine + (char) i);
        }
    }

    // Update the screen based on System.out
    public void update(boolean cursor) {
        int originalSize = lineList.size();
        ArrayList<String> displayedLineList = new ArrayList<>();
        for (int i = line; i < originalSize; i++) {
            displayedLineList.add(lineList.get(i));
        }
        if (Robotics.gamepad.inputting && cursor) {
            int size = originalSize - line;
            if (size == 0) {
                displayedLineList.add("");
                size++;
            }
            String lastLine = displayedLineList.get(size - 1);
            displayedLineList.set(size - 1, lastLine + "_");
        }
        int FONT_SIZE = 14;
        canvas = windowFrame.getCanvas();
        Graphics g = canvas.getBufferStrategy().getDrawGraphics();
        g.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        g.setFont(new Font("Courier New", Font.PLAIN, FONT_SIZE));

        int x = -horizontalDisplacement;
        int y = 14;
        for (String line : displayedLineList) {
            g.drawString(line, x, y);
            y += FONT_SIZE;
        }
        g.dispose();
        canvas.getBufferStrategy().show();
        if (gamepad.dpad_down && line < lineList.size()) {
            line++;
        }
        if (gamepad.dpad_up && line > 0) {
            line--;
        }
        if (gamepad.dpad_right) {
            horizontalDisplacement += FONT_SIZE;
        }
        if (gamepad.dpad_left && horizontalDisplacement > 0) {
            horizontalDisplacement -= FONT_SIZE;
        }
    }
}
