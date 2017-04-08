package com.mercury1089.main;

import com.mercury1089.vision.VisionPipeline;
import com.mercury1089.vision.VisionThread;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Main runner class. This is extended to suit the user's needs.
 */
public abstract class Runner {
    private static final ArrayList<VisionThread<? extends VisionPipeline>> THREADS;
    private static final Runtime RUNTIME;

    private static ExecutorService executor;

    private static boolean shutdown = false;

    static {
        // Loads our OpenCV library before anything else.
        System.loadLibrary("opencv_java310");

        // Initializes a field to access our runtime
        RUNTIME = Runtime.getRuntime();

        THREADS = new ArrayList<>();
    }

    public static void main(String[] args) {
        // Connect NetworkTables, and get access to the publishing table
        NetworkTable.setClientMode();
        // Set your team number here
        NetworkTable.setTeam(VisionConfig.team);
        // Initialize the network tables since we aren't doing this in a regular robot class
        NetworkTable.initialize();

        init();
        executor = Executors.newFixedThreadPool(THREADS.size());

        for (VisionThread<? extends VisionPipeline> p : THREADS)
            executor.execute(p);

        RUNTIME.addShutdownHook(new Thread(() -> {
            end();
            executor.shutdownNow();

            // Run a shutdown command only if the shutdown flag was ticked
            if (shutdown) {
                try {
                    RUNTIME.exec("sudo shutdown -t 5");
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }));
    }

    /**
     * Function that is run during initialization of program.
     * Use this block to initialize your cameras, create your threads, etc.
     */
    public static void init() {
        // Add implementation
    }

    /**
     * Function that is run during shutdown sequence of program.
     * Use this block to free resources, clear/save data, etc.
     */
    public static void end() {
        // Add implementation
    }

    /**
     * Adds a {@link VisionThread} to the list of threads. This has no use after the init block.
     *
     * @param thread the {@code VisionThread} to add to the list.
     */
    public static void addThread(VisionThread<? extends VisionPipeline> thread) {
        if (executor != null)
            THREADS.add(thread);
    }
}