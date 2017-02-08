import edu.wpi.cscore.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class Main {
    private static final int
            FPS = 30,
            RES_X = 320,
            RES_Y = 240;

    private static Thread gearVisionThread, highGoalThread;

    private static final Object LOCK = new Object();

    public static void main(String[] args) {
        // Loads our OpenCV library. This MUST be included
        System.loadLibrary("opencv_java310");

        // Connect NetworkTables, and get access to the publishing table
        NetworkTable.setClientMode();
        // Set your team number here
        NetworkTable.setTeam(1089);
        // Initialize the network tables since we aren't doing this in a regular robot class
        NetworkTable.initialize();

        // The root key for both vision targets
        String rootTable = "Vision/";

        // Create the thread that will process gear vision targets
        gearVisionThread = new Thread(() -> {
            NetworkTable gearVisionTable = NetworkTable.getTable(rootTable + "gearVision");
            // This stores our reference to our mjpeg server for streaming the input image
            MjpegServer inputStream = new MjpegServer("MJPEG_Pi", 1185);
            // Using the pi camera feed for this thread.
            UsbCamera camera = setUsbCamera(0, inputStream);
            camera.setResolution(RES_X, RES_Y);

            // This creates a CvSink for us to use. This grabs images from our selected camera,
            // and will allow us to use those images in opencv
            CvSink imageSink = new CvSink("CvSink_Pi");
            imageSink.setSource(camera);

            // This creates a CvSource to use. This will take in a Mat image that has had OpenCV operations
            // operations
            CvSource imageSource = new CvSource("CvSource_Pi", VideoMode.PixelFormat.kMJPEG, RES_X, RES_Y, FPS);
            MjpegServer cvStream = new MjpegServer("CvStream_Pi", 1186);
            cvStream.setSource(imageSource);

            // All Mats and Lists should be stored outside the loop to avoid allocations
            // as they are expensive to create
            Mat img = new Mat();

            // Initialize our pipeline
            MercPipeline pipeline = new MercPipeline();

            // Infinitely process image
            while (!Thread.interrupted()) {
                // Grab a frame. If it has a frame time of 0, there was an error.
                // Just skip and continue
                if (imageSink.grabFrame(img) == 0) {
                    System.out.println(imageSink.getError());
                    continue;
                }

                // Initialize variables for vision
                double
                        targetWidth = -1,
                        targetHeight = -1;

                double[] center = {-1, -1};

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
                gearVisionTable.putNumber("targetWidth", targetWidth);
                gearVisionTable.putNumber("targetHeight", targetHeight);
                gearVisionTable.putNumberArray("center", center);

                // Here is where you would write a processed image that you want to restream
                // This will most likely be a marked up image of what the camera sees
                // For now, we are just going to stream the HSV image
                imageSource.putFrame(img);
                img.release();
            }
        }, "Thread-GearVision");

        // Create the thread that will process gear vision targets
        highGoalThread = new Thread(() -> {
            NetworkTable highGoalTable = NetworkTable.getTable(rootTable + "highGoal");
            // This stores our reference to our mjpeg server for streaming the input image
            MjpegServer inputStream = new MjpegServer("MJPEG_LifeCam", 1187);
            // Using the LifeCam feed for this thread.
            UsbCamera camera = setUsbCamera(1, inputStream);
            camera.setResolution(RES_X, RES_Y);

            // This creates a CvSink for us to use. This grabs images from our selected camera,
            // and will allow us to use those images in opencv
            CvSink imageSink = new CvSink("CvSink_LifeCam");
            imageSink.setSource(camera);

            // This creates a CvSource to use. This will take in a Mat image that has had OpenCV operations
            // operations
            CvSource imageSource = new CvSource("CvSource_LifeCam", VideoMode.PixelFormat.kMJPEG, RES_X, RES_Y, FPS);
            MjpegServer cvStream = new MjpegServer("CvStream_LifeCam", 1188);
            cvStream.setSource(imageSource);

            // All Mats and Lists should be stored outside the loop to avoid allocations
            // as they are expensive to create
            Mat img = new Mat();

            // Initialize our pipeline
            MercPipeline pipeline = new MercPipeline();

            // Infinitely process image
            while (!Thread.interrupted()) {
                // Grab a frame. If it has a frame time of 0, there was an error.
                // Just skip and continue
                if (imageSink.grabFrame(img) == 0) {
                    System.out.println(imageSink.getError());
                    continue;
                }

                // Initialize variables for vision
                double
                        targetWidth = -1,
                        targetHeight = -1;

                double[] center = {-1, -1};

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

                // Here is where you would write a processed image that you want to restream
                // This will most likely be a marked up image of what the camera sees
                // For now, we are just going to stream the HSV image
                imageSource.putFrame(img);
                img.release();
            }
        }, "Thread-HighGoal");

        // Start up both threads
        gearVisionThread.start();
        highGoalThread.start();

        synchronized (LOCK) {
            while (!Thread.interrupted())
                MercPipeline.updateHSLThreshold();
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
     * @param cameraId The device id of the camera
     * @param server The {@link MjpegServer} to send the feed to
     * @return the instance of the camera
     */
    private static UsbCamera setUsbCamera(int cameraId, MjpegServer server) {
        // This gets the image from a USB camera
        // Usually this will be on device 0, but there are other overloads
        // that can be used
        UsbCamera camera = new UsbCamera("CoprocessorCamera", cameraId);
        server.setSource(camera);
        return camera;
    }
    // endregion
}