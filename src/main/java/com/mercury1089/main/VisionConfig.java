package com.mercury1089.main;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.Properties;

/**
 * Class that sets up a simple config for vision processing.
 * This is basically a wrapper for a {@link Properties} object.
 * Inside are three fields used for easy access: team, fps, resX, and resY.
 * If you want more fields, make your own properties file.
 */
public class VisionConfig {
    private static final Properties PROP;

    public static int team = 1089, fps = 15, resX = 320, resY = 240;

    static {
        PROP = new Properties();
        try {
            File config = new File("vision.properties");

            // If the config file doesn't exist, populate a properties file.
            if (!config.exists()) {
                FileWriter writer = new FileWriter(config);

                PROP.setProperty("team", "" + team);
                PROP.setProperty("fps", "" + fps);
                PROP.setProperty("res_width", "" + resX);
                PROP.setProperty("res_height", "" + resY);

                PROP.store(writer, "Vision Properties");
                writer.close();

            // Otherwise, just load it in, read the files, be done.
            } else {
                FileReader reader = new FileReader(config);
                PROP.load(reader);

                team = Integer.parseInt(PROP.getProperty("team", "" + team));
                fps = Integer.parseInt(PROP.getProperty("fps", "" + team));
                resX = Integer.parseInt(PROP.getProperty("resolution_width", "" + resX));
                resY = Integer.parseInt(PROP.getProperty("resolution_height", "" + resY));

                reader.close();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
