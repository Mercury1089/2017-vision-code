import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Calendar;

/**
 * This class creates a {@link Thread} capable of processing the input of a video feed and output contour values
 * to a network table.
 */
public class VisionThread extends Thread {
    private static final Scalar
        RED = new Scalar(0, 0, 255),
        WHITE =  new Scalar(255, 255, 255),
        BLUE = new Scalar(255, 0, 0);

    /**
     * Creates a new {@link Thread} named "VisionThread-name" with a {@link Runnable} fit for processing
     * the input from the specified {@link CvSink} using the specified {@link MercPipeline}
     * and output a processed image with contours drawn on into the specified {@link CvSource}
     * as well as into the specified {@link NetworkTable}.
     *
     * @param sink the input feed to get an image from to process
     * @param outputFeed the output feed to output the processed frame
     * @param pipeline the pipeline to use to process the image
     * @param table the network table to output values to
     * @param name the name of the thread to append to the prefix
     */
    public VisionThread(CvSink sink, CvSource outputFeed, MercPipeline pipeline, NetworkTable table, String name) {
        super (() -> {
            // All Mats and Lists should be stored outside the loop to avoid allocations
            // as they are expensive to create
            Mat img = new Mat();

            // Infinitely process image
            while (!Thread.interrupted()) {
                // Grab a frame. If it has a frame time of 0, there was an error.
                // Just skip and continue
                if (sink.grabFrame(img) == 0) {
                    System.out.println(Thread.currentThread().getName() + ": " + sink.getError());
                    continue;
                }

                // Initialize variables for vision
                double
                        targetWidth = -1,
                        targetHeight = -1;

                double[] center = {-1, -1};

                double startTime = System.currentTimeMillis();

                boolean seeTarget = false;

                // Process frame under here
                pipeline.process(img);
                ArrayList<MatOfPoint> contours = pipeline.filterContoursOutput();

                if (contours.size() == 2) {
                    Rect
                        target1 = Imgproc.boundingRect(contours.get(1)),
                        target2 = Imgproc.boundingRect(contours.get(0));

                    seeTarget = true;

                    // The first rectangle should be the leftmost rectangle
                    if (target2.x < target1.x) {
                        Rect swap;
                        swap = target1;
                        target1 = target2;
                        target2 = swap;
                    }

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

                    // Draw everything
                    Imgproc.rectangle(
                            img,
                            target1.br(),
                            target1.tl(),
                            BLUE,
                            3
                    );

                    Imgproc.rectangle(
                            img,
                            topLeft,
                            bottomRight,
                            RED,
                            3
                    );

                    Imgproc.line(
                            img,
                            new Point(targetCenter.x, targetCenter.y - 5),
                            new Point(targetCenter.x, targetCenter.y + 5),
                            RED,
                            3
                    );

                    Imgproc.line(
                            img,
                            new Point(targetCenter.x - 5, targetCenter.y),
                            new Point(targetCenter.x + 5, targetCenter.y),
                            RED,
                            3
                    );
                }

                // Draw a midpoint
                Imgproc.line(
                        img,
                        new Point(Main.RES_X / 2.0, 50),
                        new Point(Main.RES_X / 2.0, Main.RES_Y - 50),
                        WHITE,
                        1
                );

                Imgproc.line(
                        img,
                        new Point(50, Main.RES_Y / 2.0),
                        new Point(Main.RES_X - 50, Main.RES_Y / 2.0),
                        WHITE,
                        1
                );

                // Output some numbers to our network table
                table.putBoolean("seeTarget", seeTarget);
                table.putNumber("targetWidth", targetWidth);
                table.putNumber("targetHeight", targetHeight);
                table.putNumberArray("center", center);
                table.putNumber("deltaTime", System.currentTimeMillis() - startTime);
                table.putString("publishTime", Calendar.getInstance().getTime().toString());

                // Here is where you would write a processed image that you want to restream
                // This will most likely be a marked up image of what the camera sees
                // For now, we are just going to stream the HSV image
                outputFeed.putFrame(img);
                img.release();
            }
        }, "VisionThread-" + name);
    }
}
