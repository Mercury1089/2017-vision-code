package com.mercury1089.main;

import java.io.FileReader;
import java.util.Properties;

/**
 * Class that sets up a simple config for vision processing.
 * This is basically a wrapper for a {@link Properties} object.
 * Inside are three fields used for easy access: fps, resX, and resY.
 * If you want more fields, make your own properties file.
 */
public class VisionConfig {
    private static final Properties PROP;

    public static int resX, resY, fps;

    static {
        PROP = new Properties();
        try {
            FileReader reader;

            reader = new FileReader("vision.properties");
            PROP.load(reader);
            reader.close();

            fps = Integer.parseInt(PROP.getProperty("fps", "15"));
            resX = Integer.parseInt(PROP.getProperty("resolution_width", "320"));
            resY = Integer.parseInt(PROP.getProperty("resolution_height", "240"));

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
