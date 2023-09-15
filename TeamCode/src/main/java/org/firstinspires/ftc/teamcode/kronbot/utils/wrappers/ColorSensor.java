package org.firstinspires.ftc.teamcode.kronbot.utils.wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ColorSensor {
    com.qualcomm.robotcore.hardware.ColorSensor colorSensor;
    HardwareMap hardwareMap;

    public enum Color {
        RED, GREEN, BLUE, GREY
    }

    public static Integer redThreshold = 0;
    public static Integer greenThreshold = 0;
    public static Integer blueThreshold = 0;

    public ColorSensor(HardwareMap hardwareMap, String name) {
        this.hardwareMap = hardwareMap;

        colorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, name);
    }

    public int getRed() {
        return colorSensor.red();
    }

    public int getGreen() {
        return colorSensor.green();
    }

    public int getBlue() {
        return colorSensor.blue();
    }

    public int getAlpha() {
        return colorSensor.alpha();
    }

    public int getARGB() {
        return colorSensor.argb();
    }

    public Color getColor() {
        int red = getRed();
        int green = getGreen();
        int blue = getBlue();

        if (red > redThreshold && red > green && red > blue)
            return Color.RED;
        else if (green > greenThreshold && green > red && green > blue)
            return Color.GREEN;
        else if (blue > blueThreshold && blue > red && blue > green)
            return Color.BLUE;
        else
            return Color.GREY;
    }
}
