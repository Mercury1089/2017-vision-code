package com.mercury1089.vision;

import com.mercury1089.main.VisionConfig;
import edu.wpi.cscore.*;
import org.opencv.core.Mat;

/**
 * {@link Runnable} that manages one image processing process.
 * Some things have been implemented from WPI's implementation of vision processing.
 */
public class VisionThread<P extends VisionPipeline> implements Runnable {

    private static int portRaw = 1180, portOut = 5800;

    public boolean createRawStreams = true;
    public boolean createOutStreams = true;

    private final CvSink SINK;
    private final CvSource SOURCE;
    private final P PIPELINE;
    private final VisionListener<? super P> LISTENER;

    public VisionThread(String name, VideoCamera camera, P pipeline, VisionListener<? super P> listener) {
        // Set the fps and the resolution as per the config.
        camera.setFPS(VisionConfig.fps);
        camera.setResolution(VisionConfig.resX, VisionConfig.resY);
        SOURCE = new CvSource("SOURCE_" + name, VideoMode.PixelFormat.kMJPEG, VisionConfig.resX, VisionConfig.resY, VisionConfig.fps);

        // Initialize everything else.
        SINK = new CvSink("SINK_" + name);
        SINK.setSource(camera);

        PIPELINE = pipeline;

        LISTENER = listener;

        // Create streams as long as we can and want to do so.
        if (createRawStreams && portRaw <= 1190) {
            new MjpegServer("RAW_" + name, portRaw).setSource(camera);
            portRaw++;
        }

        if (createOutStreams && portOut <= 5810) {
            new MjpegServer("OUT_" + name, portOut).setSource(SOURCE);
            portOut++;
        }
    }

    public void run() {
        // Mats are heavy and very resource-heavy to instantiate,
        // so we define them outside the loop and reuse the object.
        Mat img = new Mat();

        while (!Thread.currentThread().isInterrupted()) {
            try {
                // If grabFrame returns 0, frame time is 0 and something has gone wrong.
                // Report the problem and continue.
                if (SINK.grabFrame(img) == 0) {
                    System.out.println(SINK.getError());
                    continue;
                }

                PIPELINE.process(img);
                LISTENER.run(PIPELINE);

                Thread.sleep(1000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /**
     * {@link FunctionalInterface} that is used in the thread to allow the user
     * to use the info from the pipeline from processing the image.
     */
    @FunctionalInterface
    public interface VisionListener<P extends VisionPipeline> {
        public void run(P pipeline);
    }
}
