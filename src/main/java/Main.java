import edu.wpi.cscore.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

import java.io.IOException;

/**
 * Main runner class.
 */
public class Main {
    private static final Runtime RUNTIME;
    public static final int
            FPS = 15,
            RES_X = 320,
            RES_Y = 240;

    private static Thread gearVisionThread, highGoalThread;

    private static boolean shutdown = false;

    private static final double[] DEF_THRESH = {45, 70, 140, 255, 35, 255};

    // The root key for both vision targets
    private static final String ROOT = "Vision";

    static {
        // Loads our OpenCV library before anything else.
        System.loadLibrary("opencv_java310");

        // Initializes a field to access our runtime
        RUNTIME = Runtime.getRuntime();
    }

    public static void main(String[] args) {
        // Probe for camera module driver
        try {
            RUNTIME.exec("sudo modprobe bcm2835-v4l2");
        } catch (IOException e) {
            e.printStackTrace();
        }

        // Connect NetworkTables, and get access to the publishing table
        NetworkTable.setClientMode();
        // Set your team number here
        NetworkTable.setTeam(1089);
        // Initialize the network tables since we aren't doing this in a regular robot class
        NetworkTable.initialize();

        // The network tables to use for targeting
        NetworkTable
            gearVisionTable = NetworkTable.getTable(ROOT + "/gearVision"),
            highGoalTable = NetworkTable.getTable(ROOT + "/highGoal");

        // All the Mjpeg servers to check out before and after of each feed
        // They can be found at http://roborio-1089-FRC.local:<port>
        MjpegServer
            piRawStream = new MjpegServer("RAW_Pi", 1185),
            piOutputStream = new MjpegServer("OUTPUT_Pi", 1186),
            lifeCamRawStream = new MjpegServer("RAW_LifeCam", 1187),
            lifeCamOutputStream = new MjpegServer("OUTPUT_LifeCam", 1188);

        // CvSources to take in mats with operations
        CvSource
            piSource = new CvSource("CvSource_Pi", VideoMode.PixelFormat.kMJPEG, RES_X, RES_Y, FPS),
            lifeCamSource = new CvSource("CvSource_LifeCam", VideoMode.PixelFormat.kMJPEG, RES_X, RES_Y, FPS);

        // Our usb cameras
        UsbCamera
            piCamera = new UsbCamera("Pi Camera", 1),
            lifeCam = new UsbCamera("LifeCam 3000", 0);

        // Sinks to get image feed to use for cv processing
        CvSink
            piSink = new CvSink("CvSink_Pi"),
            lifeCamSink = new CvSink("CvSink_LifeCam");

        
        FilterContourSettings gearFCS = new FilterContourSettings();
        FilterContourSettings highGoalFCS = new FilterContourSettings();

        // Pipelines to process our images
        MercPipeline
            gearPipeline = new MercPipeline(NetworkTable.getTable("Preferences").getNumberArray("hslThresholdPi", DEF_THRESH), gearFCS),
            highGoalPipeline = new MercPipeline(NetworkTable.getTable("Preferences").getNumberArray("hslThresholdLifeCam", DEF_THRESH), highGoalFCS);

        // Add listeners for values for camera settings and HSL settings
        NetworkTable.getTable(ROOT + "/gearVision").addTableListener(
    			(ITable table, String key, Object value, boolean isNew) -> {
    			    if ("brightness".equals(key))
    			        piCamera.setBrightness((int)value);
    			    else
    			        gearPipeline.updateHSL(key, (Double)value);
                }
		);

        NetworkTable.getTable(ROOT + "/highGoal").addTableListener(
                (ITable table, String key, Object value, boolean isNew) -> {
                    if ("brightness".equals(key))
                        lifeCam.setBrightness((int)value);
                    else
                        highGoalPipeline.updateHSL(key, (Double)value);
                }
		);

        // Change resolutions and framerates of cameras to be consistent.
        piCamera.setResolution(RES_X, RES_Y);
        piCamera.setFPS(FPS);
        piCamera.setBrightness(20);
        piCamera.getProperty("contrast").set(100);
        piCamera.getProperty("saturation").set(100);
        piCamera.getProperty("power_line_frequency").set(2);
        piCamera.getProperty("auto_exposure").set(1);
        piCamera.getProperty("exposure_time_absolute").set(20);

        lifeCam.setResolution(RES_X, RES_Y);
        lifeCam.setFPS(FPS);
        lifeCam.setBrightness(30);
        lifeCam.getProperty("contrast").set(100);
        lifeCam.getProperty("saturation").set(100);
        lifeCam.getProperty("white_balance_temperature_auto").set(0);
        lifeCam.getProperty("white_balance_temperature").set(10000);
        lifeCam.setExposureManual(0);

        // Set the source of the raw feed to their respective cameras
        piRawStream.setSource(piCamera);
        lifeCamRawStream.setSource(lifeCam);

        // Set sources of image sinks to get feeds from cameras
        piSink.setSource(piCamera);
        lifeCamSink.setSource(lifeCam);

        // Set sources of Mjpeg outputs to take in processed images
        piOutputStream.setSource(piSource);
        lifeCamOutputStream.setSource(lifeCamSource);

        // Create threads
        gearVisionThread = new VisionThread(piSink, piSource, gearPipeline, gearVisionTable, "gear_vision");
        highGoalThread = new VisionThread(lifeCamSink, lifeCamSource, highGoalPipeline, highGoalTable, "high_goal");

        RUNTIME.addShutdownHook(new Thread(() -> {
            System.out.println("Shutting down...");

            // Free resources
            // NOTE: I don't actually know if this works
            piOutputStream.free();
            lifeCamOutputStream.free();

            piSink.free();
            lifeCamSink.free();

            piSource.free();
            lifeCamSource.free();

            piRawStream.free();
            lifeCamRawStream.free();

            piCamera.free();
            lifeCam.free();

            // Run a shutdown command only if the shutdown flag was ticked
            if (shutdown) {
                try {
                    RUNTIME.exec("sudo shutdown -t 5");
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }));

        // Start up both threads
        gearVisionThread.start();
        highGoalThread.start();

        try {
            // Put wait methods into a loop to keep the threads from being interrupted
            // when we don't want them to be.
            NetworkTable.getTable(ROOT).putBoolean("shutdown", false);
            while(!shutdown) {
                Thread.sleep(1000);
                shutdown = NetworkTable.getTable(ROOT).getBoolean("shutdown", false);
            }

            // Shutdown the program when we're done
            System.exit(0);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}