import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
* MercPipeline class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
public class MercPipeline {

	//Outputs
	private Mat hslThresholdOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

	private final double[]
		HSL_THRESHOLD_HUE = {47.0, 95.0},
		HSL_THRESHOLD_SAT = {197.0, 255.0},
		HSL_THRESHOLD_LUM = {83.0, 195.0};

	private final FilterContourSettings FCS;

	public MercPipeline(double[] threshold, FilterContourSettings filterContourSettings) {
		HSL_THRESHOLD_HUE[0] = threshold[0];
		HSL_THRESHOLD_HUE[1] = threshold[1];
		HSL_THRESHOLD_SAT[0] = threshold[2];
		HSL_THRESHOLD_SAT[1] = threshold[3];
		HSL_THRESHOLD_LUM[0] = threshold[4];
		HSL_THRESHOLD_LUM[1] = threshold[5];
		FCS = filterContourSettings;
	}

	/**
	 * Updates a value in the HSL threshold based on the key passed in.
	 *
	 * @param key the name of the value to update
	 * @param val the value to set the specified HSL value to
	 */
	public void updateHSL(String key, double val) {
		switch(key) {
			case "hueMin":
				HSL_THRESHOLD_HUE[0] = val;
				break;
			case "hueMax":
				HSL_THRESHOLD_HUE[1] = val;
				break;
			case "satMin":
				HSL_THRESHOLD_SAT[0] = val;
				break;
			case "satMax":
				HSL_THRESHOLD_SAT[1] = val;
				break;
			case "lumMin":
				HSL_THRESHOLD_LUM[0] = val;
				break;
			case "lumMax":
				HSL_THRESHOLD_LUM[1] = val;
				break;
		}
	}

	/**
	 * Runs a {@link Mat} through the pipeline and updates the outputs.
	 *
	 * @param input the {@code Mat} to process
	 */
	public void process(Mat input) {
		// Step HSL_Threshold0:
		Mat hslThresholdInput = input;

		hslThreshold(hslThresholdInput, HSL_THRESHOLD_HUE, HSL_THRESHOLD_SAT, HSL_THRESHOLD_LUM, hslThresholdOutput);

		// Step Find_Contours0:
		Mat findContoursInput = hslThresholdOutput;
		boolean findContoursExternalOnly = false;
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

		// Step Filter_Contours0:
		ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
		//filterContours(filterContoursContours, FCS.minArea, FCS.minPerimeter, FCS.minWidth, FCS.maxWidth, FCS.minHeight, FCS.maxHeight, FCS.solidity, FCS.maxVerts, FCS.minVerts, FCS.minRatio, FCS.maxRatio, filterContoursOutput);
		filterContours(filterContoursContours, FCS.minArea, filterContoursOutput);
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
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue   The min and max hue
	 * @param sat   The min and max saturation
	 * @param lum   The min and max luminance
	 * @param out   The image in which to store the output.
	 */
	private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum,
		Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);
		Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
			new Scalar(hue[1], lum[1], sat[1]), out);
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
	 * @param minArea  the minimum area, in pixels, that any contour can be
	 * @param output   the list of {@code MatOfPoint}s that met the criteria
	 */
	private void filterContours(List<MatOfPoint> input, double minArea, List<MatOfPoint> output) {
		output.clear();
		for(MatOfPoint i : input) {
			double area = Imgproc.contourArea(i);

			if (area < minArea)
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

