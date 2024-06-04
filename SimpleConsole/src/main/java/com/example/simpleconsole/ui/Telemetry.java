package com.example.simpleconsole.ui;

import static com.example.simpleconsole.ui.Robotics.gamepad;

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

    public void update() {
        ArrayList<String> displayedLineList = new ArrayList<>(lineList.subList(line, lineList.size()));
        int FONT_SIZE = 14;
        canvas = windowFrame.getCanvas();
        Graphics g = canvas.getBufferStrategy().getDrawGraphics();
        g.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        g.setFont(new Font("Monospace", Font.PLAIN, FONT_SIZE));

        int x = 0;
        int y = 0;
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
    }
}
