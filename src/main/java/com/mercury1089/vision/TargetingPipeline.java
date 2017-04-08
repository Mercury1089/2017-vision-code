package com.mercury1089.vision;

import com.mercury1089.misc.Threshold;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * A {@link VisionPipeline} class that can process an image and find the targets on it.
 * IRL, this would be used in conjunction with, i.e.: LEDs and retroreflection of vision targets.
 */
public class TargetingPipeline implements VisionPipeline {

	// Outputs
	private Mat hslThresholdOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();

	private final Threshold
		HSL_THRESHOLD_HUE = new Threshold(47.0, 95.0),
		HSL_THRESHOLD_SAT = new Threshold(197.0, 255.0),
		HSL_THRESHOLD_LUM = new Threshold(83.0, 195.0),
		CONTOUR_THRESHOLD_AREA = new Threshold(50, 100000);

	public void updateValues(String key, Object value) {
		double val = (Double)value;

		switch(key) {
			case "hueMin":
				HSL_THRESHOLD_HUE.setMin(val);
				break;
			case "hueMax":
				HSL_THRESHOLD_HUE.setMax(val);
				break;
			case "satMin":
				HSL_THRESHOLD_SAT.setMin(val);
				break;
			case "satMax":
				HSL_THRESHOLD_SAT.setMax(val);
				break;
			case "lumMin":
				HSL_THRESHOLD_LUM.setMin(val);
				break;
			case "lumMax":
				HSL_THRESHOLD_LUM.setMax(val);
				break;
			case "areaMin":
				CONTOUR_THRESHOLD_AREA.setMin(val);
				break;
			case "areaMax":
				CONTOUR_THRESHOLD_AREA.setMax(val);
				break;
		}
	}

	public void process(Mat input) {
		// Filter HSLThreshold
		Mat hslThresholdInput = input;
		filterHSLThreshold(hslThresholdInput, hslThresholdOutput);

		// Find contours
		Mat findContoursInput = hslThresholdOutput;
		boolean findContoursExternalOnly = false;
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

		// Filter contours
		ArrayList<MatOfPoint> filterContoursList = findContoursOutput;
		filterContours(filterContoursList, filterContoursOutput);
	}

	/**
	 * This method is a generated getter for the output of a HSL_Threshold.
	 * @return Mat output from HSL_Threshold.
	 */
	public Mat hslThresholdOutput() {
		return hslThresholdOutput;
	}

	/**
	 * This method is a generated getter for the output of a Find_Contours.
	 * @return ArrayList<MatOfPoint> output from Find_Contours.
	 */
	public ArrayList<MatOfPoint> findContoursOutput() {
		return findContoursOutput;
	}

	/**
	 * This method is a generated getter for the output of a Filter_Contours.
	 * @return ArrayList<MatOfPoint> output from Filter_Contours.
	 */
	public ArrayList<MatOfPoint> filterContoursOutput() {
		return filterContoursOutput;
	}

	/**
	 * Segment an image based on hue, saturation, and luminance ranges.
	 *
	 * @param input The image on which to perform the HSLThreshold threshold.
	 * @param out   The image in which to store the output.
	 */
	private void filterHSLThreshold(Mat input, Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);

		Core.inRange(
				out,
				new Scalar(HSL_THRESHOLD_HUE.getMin(), HSL_THRESHOLD_LUM.getMin(), HSL_THRESHOLD_SAT.getMin()),
				new Scalar(HSL_THRESHOLD_HUE.getMax(), HSL_THRESHOLD_LUM.getMax(), HSL_THRESHOLD_SAT.getMax()),
				out);
	}

	/**
	 * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
	 *
	 * @param input        the image on which to perform the Distance Transform.
	 * @param externalOnly
	 * @param contours     the {@link List} to store the contours in.
	 */
	private void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		}
		else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}

	/**
	 * Filters out contours that are too small in area.
	 *
	 * @param input    the list of {@code MatOfPoint}s to filter
	 * @param output   the list of {@code MatOfPoint}s that met the criteria
	 */
	private void filterContours(List<MatOfPoint> input, List<MatOfPoint> output) {
		output.clear();
		for(MatOfPoint i : input) {
			double area = Imgproc.contourArea(i);

			if (CONTOUR_THRESHOLD_AREA.isInThreshold(area))
				continue;

			output.add(i);
		}
	}

	/**
	 * Filters out contours that do not meet certain criteria.
	 * @param inputContours is the input list of contours
	 * @param output is the the output list of contours
	 * @param minArea is the minimum area of a contour that will be kept
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept
	 * @param minWidth minimum width of a contour
	 * @param maxWidth maximum width
	 * @param minHeight minimum height
	 * @param maxHeight maximimum height
	 * @param solidity the minimum and maximum solidity of a contour
	 * @param minVertexCount minimum vertex Count of the contours
	 * @param maxVertexCount maximum vertex Count
	 * @param minRatio minimum ratio of width to height
	 * @param maxRatio maximum ratio of width to height
	 *
	 * @deprecated filterContours(1) takes less time to process as it does not
	 *             require going through unused criteria, e.g. vertex filtering.
	 */
	private void filterContours(List<MatOfPoint> inputContours, double minArea,
		double minPerimeter, double minWidth, double maxWidth, double minHeight, double
		maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
		minRatio, double maxRatio, List<MatOfPoint> output) {
		final MatOfInt hull = new MatOfInt();
		output.clear();
		//operation
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final Rect bb = Imgproc.boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth) continue;
			if (bb.height < minHeight || bb.height > maxHeight) continue;
			final double area = Imgproc.contourArea(contour);
			if (area < minArea) continue;
			if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
			Imgproc.convexHull(contour, hull);
			MatOfPoint mopHull = new MatOfPoint();
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int)hull.get(j, 0)[0];
				double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
				mopHull.put(j, 0, point);
			}
			final double solid = 100 * area / Imgproc.contourArea(mopHull);
			if (solid < solidity[0] || solid > solidity[1]) continue;
			if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
			final double ratio = bb.width / (double)bb.height;
			if (ratio < minRatio || ratio > maxRatio) continue;
			output.add(contour);
		}
	}




}

