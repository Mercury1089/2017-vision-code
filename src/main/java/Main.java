import edu.wpi.cscore.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Calendar;

public class Main {
    private static final int
            FPS = 15,
            RES_X = 320,
            RES_Y = 240;

    private static Thread gearVisionThread, highGoalThread;

    private static final Object LOCK = new Object();

    private static boolean shutdown = false;

    static {
        // Loads our OpenCV library before anything else.
        System.loadLibrary("opencv_java310");
    }

    public static void main(String[] args) {
        // The root key for both vision targets
        String rootTable = "Vision/";

        // Connect NetworkTables, and get access to the publishing table
        NetworkTable.setClientMode();
        // Set your team number here
        NetworkTable.setTeam(1089);
        // Initialize the network tables since we aren't doing this in a regular robot class
        NetworkTable.initialize();

        // The network tables to use for targeting
        NetworkTable
                gearVisionTable = NetworkTable.getTable(rootTable + "gearVision"),
                highGoalTable = NetworkTable.getTable(rootTable + "highGoal");

        // ITableListener to update values for SmartDashboard
        NetworkTable.getTable(rootTable + "hslThreshold").addTableListener((ITable source, String key, Object value, boolean isNew) -> {
            MercPipeline.updateHSLThreshold(key, (Double)value);
        });

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
        // NOTE: will not always be right; check video settings on pi
        UsbCamera
            piCamera = setUsbCamera("Pi Camera", 0, piRawStream),
            lifeCam = setUsbCamera("LifeCam 3000", 1, lifeCamRawStream);

        // Sinks to get image feed to use for cv processing
        CvSink
            piSink = new CvSink("CvSink_Pi"),
            lifeCamSink = new CvSink("CvSink_LifeCam");

        // Change resolutions to cameras to be consistent.
        piCamera.setResolution(RES_X, RES_Y);
        lifeCam.setResolution(RES_X, RES_Y);

        // Set sources of image sinks to get feeds from cameras
        piSink.setSource(piCamera);
        lifeCamSink.setSource(lifeCam);

        // Set sources of Mjpeg outputs to take in processed images
        piOutputStream.setSource(piSource);
        lifeCamOutputStream.setSource(lifeCamSource);

        // Create the thread that will process gear vision targets
        gearVisionThread = new Thread(() -> {
            // All Mats and Lists should be stored outside the loop to avoid allocations
            // as they are expensive to create
            Mat img = new Mat();

            // Initialize our pipeline
            MercPipeline pipeline = new MercPipeline();

            // Infinitely process image
            while (!Thread.interrupted()) {
                // Grab a frame. If it has a frame time of 0, there was an error.
                // Just skip and continue
                if (piSink.grabFrame(img) == 0) {
                    System.out.println(piSink.getError());
                    continue;
                }

                // Initialize variables for vision
                double
                        targetWidth = -1,
                        targetHeight = -1;

                double[] center = {-1, -1};

                double startTime = System.currentTimeMillis();

                // Process frame under here
                pipeline.process(img);
                ArrayList<MatOfPoint> contours = pipeline.filterContoursOutput();

                if (contours.size() == 2) {
                    Rect target1, target2;

                    if (Imgproc.boundingRect(contours.get(1)).x < Imgproc.boundingRect(contours.get(0)).x) {
                        target1 = Imgproc.boundingRect(contours.get(1));
                        target2 = Imgproc.boundingRect(contours.get(0));
                    } else {
                        target1 = Imgproc.boundingRect(contours.get(0));
                        target2 = Imgproc.boundingRect(contours.get(1));
                    }

                    Scalar red = new Scalar(0, 0, 255);

                    // Our targeting rect needs to encapsulate both vision targets
                    Point topLeft = new Point(
                            target1.x,
                            target1.y < target2.y ? target1.y : target2.y
                    );

                    Point bottomRight = new Point(
                            target2.x + target2.width,
                            target1.y < target2.y ? target2.y + target2.height : target1.y + target1.height
                    );

                    targetWidth = bottomRight.x - topLeft.x;
                    targetHeight = bottomRight.y - topLeft.y;

                    // Get the center of the target and check if we are centered
                    Point targetCenter = new Point(topLeft.x + targetWidth / 2, topLeft.y + targetHeight / 2);
                    center[0] = targetCenter.x;
                    center[1] = targetCenter.y;

                    // Draw target
                    Imgproc.rectangle(
                            img,
                            topLeft,
                            bottomRight,
                            red,
                            3
                    );

                    Imgproc.line(
                            img,
                            new Point(targetCenter.x, targetCenter.y - 5),
                            new Point(targetCenter.x, targetCenter.y + 5),
                            red,
                            3
                    );

                    Imgproc.line(
                            img,
                            new Point(targetCenter.x - 5, targetCenter.y),
                            new Point(targetCenter.x + 5, targetCenter.y),
                            red,
                            3
                    );
                }

                // Output some numbers to our network table
                highGoalTable.putNumber("targetWidth", targetWidth);
                highGoalTable.putNumber("targetHeight", targetHeight);
                highGoalTable.putNumberArray("center", center);
                highGoalTable.putNumber("deltaTime", System.currentTimeMillis() - startTime);
                highGoalTable.putValue("publishTime", Calendar.getInstance().getTime());

                // Here is where you would write a processed image that you want to restream
                // This will most likely be a marked up image of what the camera sees
                // For now, we are just going to stream the HSV image
                piSource.putFrame(img);
                img.release();
            }
        }, "Thread-GearVision");

        // Create the thread that will process gear vision targets
        highGoalThread = new Thread(() -> {
            // All Mats and Lists should be stored outside the loop to avoid allocations
            // as they are expensive to create
            Mat img = new Mat();

            // Initialize our pipeline
            MercPipeline pipeline = new MercPipeline();

            // Infinitely process image
            while (!Thread.interrupted()) {
                // Grab a frame. If it has a frame time of 0, there was an error.
                // Just skip and continue
                if (lifeCamSink.grabFrame(img) == 0) {
                    System.out.println(lifeCamSink.getError());
                    continue;
                }

                // Initialize variables for vision
                double
                        targetWidth = -1,
                        targetHeight = -1;

                double[] center = {-1, -1};

                double startTime = System.currentTimeMillis();

                // Process frame under here
                pipeline.process(img);
                ArrayList<MatOfPoint> contours = pipeline.filterContoursOutput();

                if (contours.size() == 2) {
                    Rect target1, target2;

                    if (Imgproc.boundingRect(contours.get(1)).x < Imgproc.boundingRect(contours.get(0)).x) {
                        target1 = Imgproc.boundingRect(contours.get(1));
                        target2 = Imgproc.boundingRect(contours.get(0));
                    } else {
                        target1 = Imgproc.boundingRect(contours.get(0));
                        target2 = Imgproc.boundingRect(contours.get(1));
                    }

                    Scalar red = new Scalar(0, 0, 255);

                    // Our targeting rect needs to encapsulate both vision targets
                    Point topLeft = new Point(
                            target1.x,
                            target1.y < target2.y ? target1.y : target2.y
                    );

                    Point bottomRight = new Point(
                            target2.x + target2.width,
                            target1.y < target2.y ? target2.y + target2.height : target1.y + target1.height
                    );

                    targetWidth = bottomRight.x - topLeft.x;
                    targetHeight = bottomRight.y - topLeft.y;

                    // Get the center of the target and check if we are centered
                    Point targetCenter = new Point(topLeft.x + targetWidth / 2, topLeft.y + targetHeight / 2);
                    center[0] = targetCenter.x;
                    center[1] = targetCenter.y;

                    // Draw target
                    Imgproc.rectangle(
                            img,
                            topLeft,
                            bottomRight,
                            red,
                            3
                    );

                    Imgproc.line(
                            img,
                            new Point(targetCenter.x, targetCenter.y - 5),
                            new Point(targetCenter.x, targetCenter.y + 5),
                            red,
                            3
                    );

                    Imgproc.line(
                            img,
                            new Point(targetCenter.x - 5, targetCenter.y),
                            new Point(targetCenter.x + 5, targetCenter.y),
                            red,
                            3
                    );
                }

                // Output some numbers to our network table
                highGoalTable.putNumber("targetWidth", targetWidth);
                highGoalTable.putNumber("targetHeight", targetHeight);
                highGoalTable.putNumberArray("center", center);
                highGoalTable.putNumber("deltaTime", System.currentTimeMillis() - startTime);
                highGoalTable.putValue("publishTime", Calendar.getInstance().getTime());

                // Here is where you would write a processed image that you want to restream
                // This will most likely be a marked up image of what the camera sees
                // For now, we are just going to stream the HSV image
                lifeCamSource.putFrame(img);
                img.release();
            }
        }, "Thread-HighGoal");

        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
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

            // Run a shutdown command.
            try {
                Runtime.getRuntime().exec("sudo shutdown -t 5");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }));

        // Start up both threads
        gearVisionThread.start();
        highGoalThread.start();

        try {
            // Put wait methods into a loop to keep the threads from being interrupted
            // when we don't want them to be.
            while(!shutdown) {
                synchronized (gearVisionThread) {
                    gearVisionThread.wait();
                }

                synchronized (highGoalThread) {
                    highGoalThread.wait();
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            System.exit(0);
        }
    }

    // region Ignore
    /**
     * Sets up an HTTP camera to get an image feed from. The camera
     * feed itself is published on a NetworkTable.
     *
     * @param cameraName The key of the camera as it appears on the NetworkTable
     * @param server The {@link MjpegServer} to send the feed to
     * @return the instance of the camera
     */
    private static HttpCamera setHttpCamera(String cameraName, MjpegServer server) {
        // Start by grabbing the camera from NetworkTables
        NetworkTable publishingTable = NetworkTable.getTable("CameraPublisher");
        // Wait for robot to connect. Allow this to be attempted indefinitely
        while (true) {
            try {
                if (publishingTable.getSubTables().size() > 0) {
                    break;
                }
                Thread.sleep(500);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        HttpCamera camera = null;
        if (!publishingTable.containsSubTable(cameraName)) {
            return null;
        }
        ITable cameraTable = publishingTable.getSubTable(cameraName);
        String[] urls = cameraTable.getStringArray("streams", null);
        if (urls == null) {
            return null;
        }
        ArrayList<String> fixedUrls = new ArrayList<String>();
        for (String url : urls) {
            if (url.startsWith("mjpg")) {
                fixedUrls.add(url.split(":", 2)[1]);
            }
        }
        camera = new HttpCamera("CoprocessorCamera", fixedUrls.toArray(new String[0]));
        server.setSource(camera);
        return camera;
    }

    /**
     * Sets up a USB camera to get an image feed from.
     *
     *
     * @param name the name to give this {@link UsbCamera}
     * @param cameraId The device id of the camera
     * @param server The {@link MjpegServer} to send the feed to
     * @return the instance of the camera
     */
    private static UsbCamera setUsbCamera(String name, int cameraId, MjpegServer server) {
        // This gets the image from a USB camera
        // Usually this will be on device 0, but there are other overloads
        // that can be used
        UsbCamera camera = new UsbCamera(name, cameraId);
        server.setSource(camera);
        return camera;
    }
    // endregion
}