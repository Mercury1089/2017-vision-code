import java.awt.Point;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.wpilibj.tables.*;
import edu.wpi.cscore.*;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class Main {
  public static void main(String[] args) {
    // Initialize our pipeline
    MercPipeline pipeline = new MercPipeline();

    // Loads our OpenCV library. This MUST be included
    System.loadLibrary("opencv_java310");

    // Connect NetworkTables, and get access to the publishing table
    NetworkTable.setClientMode();
    // Set your team number here
    NetworkTable.setTeam(1089);

    NetworkTable.initialize();


    // This is the network port you want to stream the raw received image to
    // By rules, this has to be between 1180 and 1190, so 1185 is a good choice
    int streamPort = 1185;

    // This stores our reference to our mjpeg server for streaming the input image
    MjpegServer inputStream = new MjpegServer("MJPEG Server", streamPort);

    // Selecting a Camera
    // Uncomment one of the 2 following camera options
    // The top one receives a stream from another device, and performs operations based on that
    // On windows, this one must be used since USB is not supported
    // The bottom one opens a USB camera, and performs operations on that, along with streaming
    // the input image so other devices can see it.

    // HTTP Camera
    /*
    // This is our camera name from the robot. this can be set in your robot code with the following command
    // CameraServer.getInstance().startAutomaticCapture("YourCameraNameHere");
    // "USB Camera 0" is the default if no string is specified
    String cameraName = "Vision";
    HttpCamera camera = setHttpCamera(cameraName, inputStream);
    // It is possible for the camera to be null. If it is, that means no camera could
    // be found using NetworkTables to connect to. Create an HttpCamera by giving a specified stream
    // Note if this happens, no restream will be created
    if (camera == null) {
      camera = new HttpCamera("CoprocessorCamera", "YourURLHere");
      inputStream.setSource(camera);
    }
    */
    
      

    /***********************************************/

    // USB Camera

    // This gets the image from a USB camera 
    // Usually this will be on device 0, but there are other overloads
    // that can be used
    UsbCamera camera = setUsbCamera(0, inputStream);
    // Set the resolution for our camera, since this is over USB
    camera.setResolution(640,480);

    // This creates a CvSink for us to use. This grabs images from our selected camera, 
    // and will allow us to use those images in opencv
    CvSink imageSink = new CvSink("CV Image Grabber");
    imageSink.setSource(camera);

    // This creates a CvSource to use. This will take in a Mat image that has had OpenCV operations
    // operations 
    CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 640, 480, 30);
    MjpegServer cvStream = new MjpegServer("CV Image Stream", 1186);
    cvStream.setSource(imageSource);

    // All Mats and Lists should be stored outside the loop to avoid allocations
    // as they are expensive to create
    Mat inputImage = new Mat();
    
    // Create a network table for publishing target values
    NetworkTable visionTable = NetworkTable.getTable("vision");

    // Infinitely process image
    while (true) {
      // Initialize variables for vision
      double
      	targetWidth = -1,
      	targetHeight = -1;
      
      double[] center = {-1, -1};
    	
      // Grab a frame. If it has a frame time of 0, there was an error.
      // Just skip and continue
      if (imageSink.grabFrame(inputImage) == 0) {
        System.out.println("Please stop the pain");
      	continue;
      }

      // Process frame under here		
	pipeline.process(inputImage);
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
		org.opencv.core.Point topLeft = new org.opencv.core.Point(
				target1.x, 
				target1.y < target2.y ? target1.y : target2.y
		);
		
		org.opencv.core.Point bottomRight = new org.opencv.core.Point(
				target2.x + target2.width, 
				target1.y < target2.y ? target2.y + target2.height : target1.y + target1.height
		);
	
		targetWidth = bottomRight.x - topLeft.x;
		targetHeight = bottomRight.y - topLeft.y;
		
		// Get the center of the target and check if we are centered
		org.opencv.core.Point targetCenter = new org.opencv.core.Point(topLeft.x + targetWidth / 2, topLeft.y + targetHeight / 2);
		center[0] = targetCenter.x;
		center[1] = targetCenter.y;
		
		// Draw target
		Imgproc.rectangle(
				inputImage, 
				topLeft, 
				bottomRight, 
				red, 
				3
		);
		
		Imgproc.line(
				inputImage, 
				new org.opencv.core.Point(targetCenter.x, targetCenter.y - 5), 
				new org.opencv.core.Point(targetCenter.x, targetCenter.y + 5), 
				red, 
				3
		);
		
		Imgproc.line(
				inputImage, 
				new org.opencv.core.Point(targetCenter.x - 5, targetCenter.y), 
				new org.opencv.core.Point(targetCenter.x + 5, targetCenter.y), 
				red, 
				3
		);
	}
	
	  // Give the output stream a new image to display
    visionTable.putNumber("targetWidth", targetWidth);
    visionTable.putNumber("targetHeight", targetHeight);
    visionTable.putNumberArray("center", center);
    
      // Here is where you would write a processed image that you want to restreams
      // This will most likely be a marked up image of what the camera sees
      // For now, we are just going to stream the HSV image
      imageSource.putFrame(inputImage);
    }
  }

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
            // TODO Auto-generated catch block
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

  private static UsbCamera setUsbCamera(int cameraId, MjpegServer server) {
    // This gets the image from a USB camera 
    // Usually this will be on device 0, but there are other overloads
    // that can be used
    UsbCamera camera = new UsbCamera("CoprocessorCamera", cameraId);
    server.setSource(camera);
    return camera;
  }
}