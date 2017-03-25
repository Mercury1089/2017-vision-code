/**
 * This class encapsulates all the settings used when filtering contours.
 * This should be used on a per-pipeline basis.
 */
public class FilterContourSettings {
	public double minArea = 50.0;
	public double minPerimeter = 0;
	public double minWidth = 0;
	public double maxWidth = 1000;
	public double minHeight = 0;
	public double maxHeight = 1000;
	public double[] solidity = {80, 100};
	public double maxVerts = 1000000;
	public double minVerts = 0;
	public double minRatio = 0;
	public double maxRatio = 1000;
}