package com.wilyworks.simulator.framework;

import static java.awt.Font.MONOSPACED;
import static java.awt.font.TextAttribute.POSTURE_REGULAR;
import static java.awt.font.TextAttribute.WEIGHT_REGULAR;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.robotcore.external.Telemetry.DisplayFormat;

import androidx.annotation.Nullable;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Paint;
import java.awt.font.FontRenderContext;
import java.awt.font.LineBreakMeasurer;
import java.awt.font.TextAttribute;
import java.awt.font.TextLayout;
import java.math.BigInteger;
import java.text.AttributedCharacterIterator;
import java.text.AttributedString;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Helper class for handling simplified HTML display.
 */
class Layout {
    public static final int TELEMETRY_WIDTH = 237; // Line width, in pixel units
    public static final int TELEMETRY_HEIGHT = 237;
    private static final int FONT_SIZE = 10; // Default font size
    private static final float[] HEADING_MULTIPLES = { // Heading size multiples
            1.5f, 1.4f, 1.3f, 1.2f, 1.1f, 1f,
    };

    // \n
    private final Pattern newlineSearchPattern = Pattern.compile("(\\n)");

    // <big>, &nbsp;, \n
    // For a tag, group2 = element name, group3 = arguments (needs trimming)
    private final Pattern htmlSearchPattern = Pattern.compile("(\\n|&.*?;|<([/\\w]+)(.*?)>)");

    // style='color: 0xffffff; background: 0x3e3e3e;'
    private final Pattern spanColorPattern = Pattern.compile(
            "\\s*?style\\s*?=\\s*?['|\"].*?color\\s*?:\\s*?(?:0x|#)([0-9a-fA-F]+)");
    private final Pattern spanBackgroundPattern = Pattern.compile(
            "\\s*?style\\s*?=\\s*?['|\"].*?background\\s*?:\\s*?(?:0x|#)([0-9a-fA-F]+)");

    // <font color='#00ff00'>
    private final Pattern fontColorPattern = Pattern.compile(
            "\\s*?color\\s*?=\\s*?['|\"](?:0x|#)([0-9a-fA-F]+)");

    // Track tag attributes as we accumulate the string buffer:
    static class Attribute {
        AttributedCharacterIterator.Attribute attribute; // addAttribute() attribute
        Object value; // addAttribute() value
        int pos; // Position in the buffer

        public Attribute(AttributedCharacterIterator.Attribute attribute, Object value, int pos) {
            this.attribute = attribute; this.value = value; this.pos = pos;
        }
    }

    // Track line breaks as we accumulate the string buffer:
    static class LineBreak {
        int pos; // Position in the buffer
        float lineCount; // Count of lines to advance, can be fractional

        public LineBreak(int pos, float lineCount) {
            this.pos = pos; this.lineCount = lineCount;
        }
    }

    // List of supported HTML entities and their translations:
    static final Map<String, String> ENTITY_MAP = new HashMap<String, String>() {{
        put("&nbsp;", " ");
        put("&ensp;", "  ");
        put("&emsp;", "    ");
        put("&lt;", "<");
        put("&gt;", ">");
        put("&amp;", "&");
        put("&quot;", "\"");
        put("&apos;", "'");
    }};

    // Returns true if the current line is empty:
    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    private boolean isEmptyLine(StringBuilder builder, List<LineBreak> lineBreaks) {
        if ((builder.length() == 0) || (lineBreaks.isEmpty()))
            return true;

        LineBreak lastBreak = lineBreaks.get(lineBreaks.size() - 1);
        String lastLine = builder.substring(lastBreak.pos).trim();
        return lastLine.isEmpty();
    }

    // Render what we've accumulated:
    private void renderText(Graphics2D graphics, StringBuilder buffer, List<Attribute> attributes, List<LineBreak> lineBreaks) {
        String text = buffer.toString();

        // Convert our accumulation buffer to an AttributeString:
        AttributedString attributedString = new AttributedString(text);
        for (Attribute attribute : attributes) {
            if (attribute.pos != text.length())
                attributedString.addAttribute(attribute.attribute, attribute.value, attribute.pos, text.length());
        }

        // Render with word wrapping:
        LineBreakMeasurer measurer = new LineBreakMeasurer(
                attributedString.getIterator(),
                new FontRenderContext(null, true, true));

        Iterator<LineBreak> iterator = lineBreaks.iterator();
        LineBreak lineBreak = iterator.next();
        float yCurrent = 0;

        while (true) {
            // Find where the measurer wants to put the next line break and compare that to
            // where we want to put the next line break:
            int layoutEndPos = measurer.nextOffset(TELEMETRY_WIDTH);
            float extraLineCount = 0;

            if (lineBreak.pos <= layoutEndPos) {
                // We always advance by one line so start the count negative:
                extraLineCount = -1;

                // If the explicitly requested newline comes before the natural layout line-end,
                // then make that the new line. Use a loop to handle multiple explicit newlines
                // in a row:
                while (lineBreak.pos <= layoutEndPos) {
                    layoutEndPos = lineBreak.pos;
                    extraLineCount += lineBreak.lineCount;
                    lineBreak = iterator.next();
                }
            }

            TextLayout layout = measurer.nextLayout(TELEMETRY_WIDTH, layoutEndPos, false);
            if (layout == null)
                break;

            // Handle y offset for the line:
            yCurrent += layout.getAscent();
            layout.draw(graphics, 0, yCurrent);
            yCurrent += layout.getDescent();

            // Handle extra lines:
            yCurrent += extraLineCount * (layout.getAscent() + layout.getDescent());
        }
    }

    // History structure for tags:
    static public class ColorRecord {
        Paint foreground;
        Paint background;

        public ColorRecord(Paint foregroundColor, Paint backgroundColor) {
            this.foreground = foregroundColor; this.background = backgroundColor;
        }
    }

    // Workhorse routine to parse the HTML and render it:
    public void parseAndRender(Graphics2D graphics, DisplayFormat format, List<String> lines) {
        StringBuilder buffer= new StringBuilder();
        ArrayList<Attribute> attributes = new ArrayList<>();
        ArrayList<LineBreak> lineBreaks = new ArrayList<>();

        StringBuilder textBuilder = new StringBuilder();
        for (String line: lines) {
            textBuilder.append(line);
            textBuilder.append("\n");
        }
        String text = textBuilder.toString();

// text = "This\n\nis a <b>long</b> and <big><big>big</big></big> text that <strike>needs</strike> to be wrapped into multiple lines.   \n\n" + "<tt>We want to handle line breaks gracefully.</tt>";

        Pattern searchPattern = (format == DisplayFormat.HTML) ? htmlSearchPattern : newlineSearchPattern;

        LinkedList<String> familyStack = new LinkedList<>();
        LinkedList<Float> weightStack = new LinkedList<>();
        LinkedList<Float> postureStack = new LinkedList<>();
        LinkedList<Float> sizeStack = new LinkedList<>();
        LinkedList<Integer> superscriptStack = new LinkedList<>();
        LinkedList<ColorRecord> colorStack = new LinkedList<>();
        LinkedList<Integer> underlineStack = new LinkedList<>();
        LinkedList<Boolean> strikeThroughStack = new LinkedList<>();

        String family = (format == DisplayFormat.MONOSPACE) ? MONOSPACED : "Default";
        float weight = WEIGHT_REGULAR;
        float posture = POSTURE_REGULAR;
        float size = FONT_SIZE;
        int superscript = 0;
        Paint foreground = new Color(0xffffff); // The standard text color
        Paint background = new Color(0x390708); // The reddish-brown of the driver station
        int underline = -1;
        boolean strikeThrough = false;

        attributes.add(new Attribute(TextAttribute.FAMILY, family, 0));
        attributes.add(new Attribute(TextAttribute.WEIGHT, weight, 0));
        attributes.add(new Attribute(TextAttribute.POSTURE, posture, 0));
        attributes.add(new Attribute(TextAttribute.SIZE, size, 0));
        attributes.add(new Attribute(TextAttribute.SUPERSCRIPT, superscript, 0));
        attributes.add(new Attribute(TextAttribute.FOREGROUND, foreground, 0));
        attributes.add(new Attribute(TextAttribute.BACKGROUND, background, 0));
        attributes.add(new Attribute(TextAttribute.UNDERLINE, underline, 0));
        //noinspection ConstantValue
        attributes.add(new Attribute(TextAttribute.STRIKETHROUGH, strikeThrough, 0));

        // Clear the background using the default background color:
        graphics.setPaint(background);
        graphics.fillRect(0, 0, 10000, 10000);
        graphics.setClip(0, 0, TELEMETRY_WIDTH, TELEMETRY_HEIGHT);

        // Start parsing!
        Matcher matcher = searchPattern.matcher(text);
        int previousSearchEnd = 0;
        while (matcher.find()) {
            buffer.append(text, previousSearchEnd, matcher.start());
            previousSearchEnd = matcher.end();
            switch (matcher.group().charAt(0)) {
                case '\n':
                    buffer.append(" "); // Prevent the line breaker from merging the next line
                    lineBreaks.add(new LineBreak(buffer.length(), 1));
                    break;

                case '&':
                    String entitySubstitution = ENTITY_MAP.get(matcher.group());
                    if (entitySubstitution != null) {
                        buffer.append(entitySubstitution);
                    }
                    break;

                case '<':
                    String element = matcher.group(2);
                    String tagArguments = matcher.group(3);
                    switch (element) {
                        case "div":
                        case "p":
                            if (!isEmptyLine(buffer, lineBreaks)) {
                                buffer.append(" "); // Prevent the line breaker from merging the next line
                                lineBreaks.add(new LineBreak(buffer.length(), 1));
                            }
                            break;
                        case "br":
                        case "/div":
                        case "/p":
                            buffer.append(" "); // Prevent the line breaker from merging the next line
                            lineBreaks.add(new LineBreak(buffer.length(), 1));
                            break;

                        case "h1": case "h2": case "h3": case "h4": case "h5": case "h6":
                            sizeStack.push(size);
                            size *= HEADING_MULTIPLES[element.charAt(1) - '1'];
                            attributes.add(new Attribute(TextAttribute.SIZE, size, buffer.length()));
                            if (!isEmptyLine(buffer, lineBreaks)) {
                                buffer.append(" "); // Prevent the line breaker from merging the next line
                                lineBreaks.add(new LineBreak(buffer.length(), 1));
                            }
                            break;
                        case "/h1": case "/h2": case "/h3": case "/h4": case "/h5": case "/h6":
                            if (!sizeStack.isEmpty()) {
                                size = sizeStack.pop();
                                buffer.append(" "); // Prevent the line breaker from merging the next line
                                attributes.add(new Attribute(TextAttribute.SIZE, size, buffer.length()));
                                lineBreaks.add(new LineBreak(buffer.length(), 1.5f));
                            }
                            break;

                        // Text attributes:
                        case "span":
                            colorStack.push(new ColorRecord(foreground, background));
                            Matcher spanColorMatch = spanColorPattern.matcher(tagArguments);
                            if (spanColorMatch.find()) {
                                BigInteger rgb = new BigInteger(spanColorMatch.group(1), 16);
                                foreground = new Color(rgb.intValue());
                                attributes.add(new Attribute(TextAttribute.FOREGROUND, foreground, buffer.length()));
                            }
                            Matcher spanBackgroundMatch = spanBackgroundPattern.matcher(tagArguments);
                            if (spanBackgroundMatch.find()) {
                                BigInteger rgb = new BigInteger(spanBackgroundMatch.group(1), 16);
                                background = new Color(rgb.intValue());
                                attributes.add(new Attribute(TextAttribute.BACKGROUND, background, buffer.length()));
                            }
                            break;
                        case "font":
                            colorStack.push(new ColorRecord(foreground, background));
                            Matcher fontColorMatch = fontColorPattern.matcher(tagArguments);
                            if (fontColorMatch.find()) {
                                BigInteger rgb = new BigInteger(fontColorMatch.group(1), 16);
                                foreground = new Color(rgb.intValue());
                                attributes.add(new Attribute(TextAttribute.FOREGROUND, foreground, buffer.length()));
                            }
                            break;
                        case "/span":
                        case "/font":
                            if (!colorStack.isEmpty()) {
                                ColorRecord node = colorStack.pop();
                                foreground = node.foreground;
                                background = node.background;
                                attributes.add(new Attribute(TextAttribute.FOREGROUND, foreground, buffer.length()));
                                attributes.add(new Attribute(TextAttribute.BACKGROUND, background, buffer.length()));
                            }
                            break;

                        case "big":
                            sizeStack.push(size);
                            size *= 1.25f;
                            attributes.add(new Attribute(TextAttribute.SIZE, size, buffer.length()));
                            break;
                        case "small":
                            sizeStack.push(size);
                            size *= 0.8f;
                            attributes.add(new Attribute(TextAttribute.SIZE, size, buffer.length()));
                            break;
                        case "/small":
                        case "/big":
                            if (!sizeStack.isEmpty()) {
                                size = sizeStack.pop();
                                attributes.add(new Attribute(TextAttribute.SIZE, size, buffer.length()));
                            }
                            break;

                        case "tt":
                            familyStack.push(family);
                            family = MONOSPACED;
                            attributes.add(new Attribute(TextAttribute.FAMILY, family, buffer.length()));
                            break;
                        case "/tt":
                            if (!familyStack.isEmpty()) {
                                family = familyStack.pop();
                                attributes.add(new Attribute(TextAttribute.FAMILY, family, buffer.length()));
                            }
                            break;

                        case "b":
                            weightStack.push(weight);
                            weight = TextAttribute.WEIGHT_BOLD;
                            attributes.add(new Attribute(TextAttribute.WEIGHT, weight, buffer.length()));
                            break;
                        case "/b":
                            if (!weightStack.isEmpty()) {
                                weight = weightStack.pop();
                                attributes.add(new Attribute(TextAttribute.WEIGHT, weight, buffer.length()));
                            }
                            break;

                        case "i":
                            postureStack.push(posture);
                            posture = TextAttribute.POSTURE_OBLIQUE;
                            attributes.add(new Attribute(TextAttribute.POSTURE, posture, buffer.length()));
                            break;
                        case "/i":
                            if (!postureStack.isEmpty()) {
                                posture = postureStack.pop();
                                attributes.add(new Attribute(TextAttribute.POSTURE, posture, buffer.length()));
                            }
                            break;

                        case "u":
                            underlineStack.push(underline);
                            underline = TextAttribute.UNDERLINE_ON;
                            attributes.add(new Attribute(TextAttribute.UNDERLINE, underline, buffer.length()));
                            break;
                        case "/u":
                            if (!underlineStack.isEmpty()) {
                                underline = underlineStack.pop();
                                attributes.add(new Attribute(TextAttribute.UNDERLINE, underline, buffer.length()));
                            }
                            break;

                        case "strike":
                            strikeThroughStack.push(strikeThrough);
                            strikeThrough = TextAttribute.STRIKETHROUGH_ON;
                            attributes.add(new Attribute(TextAttribute.STRIKETHROUGH, strikeThrough, buffer.length()));
                            break;
                        case "/strike":
                            if (!strikeThroughStack.isEmpty()) {
                                strikeThrough = strikeThroughStack.pop();
                                attributes.add(new Attribute(TextAttribute.STRIKETHROUGH, strikeThrough, buffer.length()));
                            }
                            break;

                        case "sup":
                            superscriptStack.push(superscript);
                            superscript = TextAttribute.SUPERSCRIPT_SUPER;
                            attributes.add(new Attribute(TextAttribute.SUPERSCRIPT, superscript, buffer.length()));
                            break;
                        case "sub":
                            superscriptStack.push(superscript);
                            superscript = TextAttribute.SUPERSCRIPT_SUB;
                            attributes.add(new Attribute(TextAttribute.SUPERSCRIPT, superscript, buffer.length()));
                            break;
                        case "/sup":
                        case "/sub":
                            if (!superscriptStack.isEmpty()) {
                                superscript = superscriptStack.pop();
                                attributes.add(new Attribute(TextAttribute.SUPERSCRIPT, superscript, buffer.length()));
                            }
                            break;
                    }
            }
        }

        buffer.append(text, previousSearchEnd, text.length());

        // The layout code can't take empty strings:
        if (buffer.length() > 0) {
            lineBreaks.add(new LineBreak(Integer.MAX_VALUE, 0)); // Terminator
            renderText(graphics, buffer, attributes, lineBreaks);
        }
    }
}

class WilyItem implements Telemetry.Item {
    WilyTelemetry telemetry;

    WilyItem(WilyTelemetry telemetry) {
        this.telemetry = telemetry;
    }
    @Override
    public String getCaption() {
        return "";
    }

    @Override
    public Telemetry.Item setCaption(String caption) {
        return null;
    }

    @Override
    public Telemetry.Item setValue(String format, Object... args) {
        return null;
    }

    @Override
    public Telemetry.Item setValue(Object value) {
        return null;
    }

    @Override
    public <T> Telemetry.Item setValue(Func<T> valueProducer) {
        return null;
    }

    @Override
    public <T> Telemetry.Item setValue(String format, Func<T> valueProducer) {
        return null;
    }

    @Override
    public Telemetry.Item setRetained(@Nullable Boolean retained) {
        return this;
    }

    @Override
    public boolean isRetained() {
        return false;
    }

    @Override
    public Telemetry.Item addData(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
        return this;
    }

    @Override
    public Telemetry.Item addData(String caption, Object value) {
        telemetry.addData(caption, value);
        return this;
    }

    @Override
    public <T> Telemetry.Item addData(String caption, Func<T> valueProducer) {
        return null;
    }

    @Override
    public <T> Telemetry.Item addData(String caption, String format, Func<T> valueProducer) {
        return null;
    }
}

class WilyLine implements Telemetry.Line {
    WilyTelemetry telemetry;
    WilyLine(WilyTelemetry telemetry) { this.telemetry = telemetry; }

    @Override
    public Telemetry.Item addData(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
        return new WilyItem(telemetry);
    }

    @Override
    public Telemetry.Item addData(String caption, Object value) {
        telemetry.addData(caption, value);
        return new WilyItem(telemetry);
    }

    @Override
    public <T> Telemetry.Item addData(String caption, Func<T> valueProducer) {
        return null;
    }

    @Override
    public <T> Telemetry.Item addData(String caption, String format, Func<T> valueProducer) {
        return null;
    }
}

/**
 * This class implements a lightweight emulation of FTC Telemetry that can run on the PC.
 */
public class WilyTelemetry implements Telemetry {
    final int MAX_LINES = 36; // It's 18 with a regular sized font

    // Global state:
    public static WilyTelemetry instance;

    // Class state:
    TelemetryWindow telemetryWindow;
    Canvas canvas;
    ArrayList<String> lineList = new ArrayList<>();
    DisplayFormat displayFormat = DisplayFormat.CLASSIC; // HTML vs. monospace modes
    Layout layout = new Layout();

    // Wily Works constructor for a Telemetry object:
    public WilyTelemetry(java.awt.Image icon) {
        instance = this;
        telemetryWindow = new TelemetryWindow("Telemetry",
                Layout.TELEMETRY_WIDTH + 5, Layout.TELEMETRY_HEIGHT + 5);
        telemetryWindow.setIconImage(icon);
        telemetryWindow.setVisible(true);
        canvas = telemetryWindow.getCanvas();
    }

    public Line addLine(String string) {
        if (lineList.size() <= MAX_LINES) {
            int newLineIndex;
            while ((newLineIndex = string.indexOf("\n")) != -1) {
                String line = string.substring(0, newLineIndex);
                lineList.add(line);
                string = string.substring(newLineIndex + 1);
            }
            lineList.add(string);
        }
        return new WilyLine(this);
    }

    @Override
    public boolean removeLine(Line line) {
        return false;
    }

    @Override
    public boolean isAutoClear() {
        return false;
    }

    @Override
    public void setAutoClear(boolean autoClear) { }

    @Override
    public int getMsTransmissionInterval() {
        return 0;
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) { }

    @Override
    public String getItemSeparator() {
        return null;
    }

    @Override
    public void setItemSeparator(String itemSeparator) { }

    @Override
    public String getCaptionValueSeparator() {
        return null;
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) { }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        this.displayFormat = displayFormat;
    }

    @Override
    public Log log() {
        return null;
    }

    public Item addData(String caption, Object value) {
        addLine(String.format("%s : %s", caption, value.toString()));
        return null; // ###
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        return null;
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return null;
    }

    @Override
    public boolean removeItem(Item item) {
        return false;
    }

    public Item addData(String caption, String format, Object... args) {
        addData(caption, String.format(format, args));
        return null; // ###
    }

    public Line addLine() { return addLine(""); }
    public void clear() { lineList.clear(); }
    public void clearAll() { lineList.clear(); }

    @Override
    public Object addAction(Runnable action) {
        return null;
    }

    @Override
    public boolean removeAction(Object token) {
        return false;
    }

    @Override
    public void speak(String text) { }

    @Override
    public void speak(String text, String languageCode, String countryCode) { }

    public boolean update() {
        Graphics2D g = (Graphics2D) canvas.getBufferStrategy().getDrawGraphics();
        g.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        layout.parseAndRender(g, displayFormat, lineList);
        g.dispose();

        canvas.getBufferStrategy().show();
        lineList.clear();
        return true;
    }
}
