package com.mercury1089.vision;

import org.opencv.core.Mat;

/**
 * Interface that just allows pipelines to share a common "input" method.
 * Implement this class if you are using a pipeline class.
 */
public interface VisionPipeline {
    /**
     * Processes the given {@link Mat} and updates the outputs.
     *
     * @param source the {@code Mat} to process
     */
    public void process(Mat source);

    /**
     * Updates a specified value in the pipeline.
     *
     * @param key   the value to update
     * @param value the value to update the specified key to
     */
    public void updateValues(String key, Object value);
}
