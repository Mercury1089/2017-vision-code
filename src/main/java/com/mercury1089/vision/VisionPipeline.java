package com.mercury1089.vision;

import org.opencv.core.Mat;

/**
 * Interface that just allows pipelines to share a common "input" method.
 * Implement this class if you are using a pipeline class.
 */
public interface VisionPipeline {
    public void process(Mat source);
}
