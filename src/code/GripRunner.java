package code;

/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

/**
 * A GRIP runner is a convenient wrapper object to make it easy to run vision
 * pipelines from robot code.
 *
 * @see VisionPipeline
 */
public class GripRunner<P extends GripPipeline> {

	static {
//		System.out.println(System.getProperty("java.library.path"));
//		System.out.println("");
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
	
	public static final boolean DEBUG = Boolean.valueOf(System.getProperty("debug", "false"));
	public static final boolean HEADLESS = Boolean.valueOf(System.getProperty("headless", "false"));
	final static int FRAME_COUNT_SPAN = 3;

	private final VideoCapture m_cvSink;
	private final P m_pipeline;
	private final Mat m_image = new Mat();
	private final Listener<? super P> m_listener;
	private int frameCount = 0;
	private long frameCountTimeout = System.currentTimeMillis() + 1000L * FRAME_COUNT_SPAN;
	
	NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    NetworkTableEntry xEntry = table.getEntry("horizontalAngle");

	/**
	 * Listener interface for a callback that should run after a pipeline has
	 * processed its input.
	 *
	 * @param <P> the type of the pipeline this listener is for
	 */
	@FunctionalInterface
	public interface Listener<P extends GripPipeline> {

		/**
		 * Called when the pipeline has run. This shouldn't take much time to
		 * run because it will delay later calls to the pipeline's
		 * {@link VisionPipeline#process process} method. Copying the outputs
		 * and code that uses the copies should be <i>synchronized</i> on the
		 * same mutex to prevent multiple threads from reading and writing to
		 * the same memory at the same time.
		 *
		 * @param pipeline
		 *            the vision pipeline that ran
		 */
		void copyPipelineOutputs(P pipeline);

	}

	/**
	 * Creates a new vision runner. It will take images from the
	 * {@code videoSource}, send them to the {@code pipeline}, and call the
	 * {@code listener} when the pipeline has finished to alert user code when
	 * it is safe to access the pipeline's outputs.
	 *
	 * @param camera the video source to use to supply images for the pipeline
	 * @param pipeline the vision pipeline to run
	 * @param listener a function to call after the pipeline has finished running
	 */
	public GripRunner(VideoCapture camera, P pipeline, Listener<? super P> listener) {
		this.m_pipeline = pipeline;
		this.m_listener = listener;
		m_cvSink = camera;
	}

	/**
	 * Runs the pipeline one time, giving it the next image from the video
	 * source specified in the constructor. This will block until the source
	 * either has an image or throws an error. If the source successfully
	 * supplied a frame, the pipeline's image input will be set, the pipeline
	 * will run, and the listener specified in the constructor will be called to
	 * notify it that the pipeline ran.
	 *
	 * <p>
	 * This method is exposed to allow teams to add additional functionality or
	 * have their own ways to run the pipeline. Most teams, however, should just
	 * use {@link #runForever} in its own thread using a {@link VisionThread}.
	 * </p>
	 */
	public void runOnce() {
		m_cvSink.read(m_image);
		m_pipeline.process(m_image);
		m_pipeline.drawCenters(m_pipeline.findContourCenters(m_pipeline.filterContoursOutput()), m_pipeline.findGoalCenter(m_pipeline.findGoal(m_pipeline.filterContoursOutput())), m_image);
		//m_pipeline.drawMyContours(m_pipeline.approxContours(m_pipeline.findContoursOutput()), m_image);
		m_pipeline.drawMyContours(m_pipeline.approxContours(m_pipeline.filterContoursOutput()), m_pipeline.findGoal(m_pipeline.filterContoursOutput()), m_image);
		
		if (m_listener != null) { 
			m_listener.copyPipelineOutputs(m_pipeline); 
		}
	}

	/**
	 * A convenience method that calls {@link #runOnce()} in an infinite loop.
	 * This must be run in a dedicated thread, and cannot be used in the main
	 * robot thread because it will freeze the robot program.
	 * 
	 * @see VisionThread
	 */
	public void runForever() {
		inst.startClientTeam(2399);  // where TEAM=190, 294, etc, or use inst.startClient("hostname") or similar
	    inst.startDSClient();  // recommended if running on DS computer; this gets the robot IP from the DS
	    
		while (!Thread.currentThread().isInterrupted()) {
			runOnce();
			runNetworkTable();
			if (DEBUG)  {
				frameCount++;	
				if (System.currentTimeMillis() > frameCountTimeout)  {
					System.out.println("fps: " + (frameCount / FRAME_COUNT_SPAN));
					frameCount = 0;
					frameCountTimeout = System.currentTimeMillis() + 1000L * FRAME_COUNT_SPAN;
				}
			}
		}
	}
	
	public void runNetworkTable() {
		double xAngle = m_pipeline.getXAngle();
		xEntry.setDouble(xAngle);
	}
	

	/**
	 * Make a connection to a camera.
	 * 
	 * @param device  Camera number.
	 * @param width  Window width in pixels.
	 * @param height Window height in pixels.
	 * @param exposure Relative exposure.
	 * @return
	 */
	public static VideoCapture makeCamera(int device, int width, int height, double exposure) {
		VideoCapture camera = new VideoCapture(1);
		camera.set(Videoio.CAP_PROP_FRAME_WIDTH, width);
		camera.set(Videoio.CAP_PROP_FRAME_HEIGHT, height);
		if (exposure > -1.0) {
			System.out.println("\t" + exposure);
			camera.set(Videoio.CAP_PROP_AUTO_EXPOSURE, 0);
			camera.set(Videoio.CAP_PROP_EXPOSURE, exposure);
		}
		if (!camera.isOpened()) {
			throw new RuntimeException("Camera will not open");
		}
		return camera;
	}
	
	/**
	 * Make a GUI window on which to display a {@link Mat} image.
	 * 
	 * @param title Title on the window.
	 * @param width  Window width in pixels.
	 * @param height Window height in pixels.
	 * @return a new {@link VideoViewer}
	 */
	public static VideoViewer makeWindow(String title, int width, int height)  {
		if (HEADLESS)  { return null; }
		return new VideoViewer("GRIP", width, height);
	}

//	public ArrayList<int[]> findContourCenters(ArrayList<MatOfPoint> contours) {
//		
//		ArrayList<Moments> moments = new ArrayList<Moments>(contours.size());
//		ArrayList<int[]> centers = new ArrayList<int[]>();
//		
//		ArrayList<Integer> xcenters = new ArrayList<Integer>(contours.size());
//		ArrayList<Integer> ycenters = new ArrayList<Integer>(contours.size());
//		
//		for(int i = 0; i < contours.size(); i++) {
//			moments.add(i, Imgproc.moments(contours.get(i), false));
//			Moments p = moments.get(i);
//			int[] xy = {(int) (p.get_m10() / p.get_m00()), (int) (p.get_m01() / p.get_m00())};
//			centers.add(i, xy);
//		}
//		
//		return centers;
//	}
//	
//	public void drawMyContours(List<MatOfPoint> contours, Mat img) {
//		Scalar s = new Scalar(255, 0, 0, 1);
//		Imgproc.drawContours(img, contours, -1, s, 1);
//	}
//	
//	public void drawCenters(ArrayList<int[]> centers, Mat img) {
//		for (int i = 0; i < centers.size(); i++) {
//			int[] xy = centers.get(i);
//			Point p = new Point(xy[0], xy[1]);
//			Scalar s = new Scalar(0, 255, 0, 1);
//			
//			Imgproc.drawMarker(img, p, s);
//		}
//	}
//	
//	public ArrayList<MatOfPoint> approxContours(ArrayList<MatOfPoint> contours) {
//		
//		ArrayList<MatOfPoint2f> contours2F = mopToMOP2F(contours);
//		ArrayList<MatOfPoint2f> newContours2F = new ArrayList<MatOfPoint2f>();
//		
//		for(int i = 0; i < contours.size(); i++) {			
//			MatOfPoint2f input = contours2F.get(i);
//			MatOfPoint2f output = new MatOfPoint2f();
//			
//			Imgproc.approxPolyDP(input, output, 3, true);
//			
//			newContours2F.add(output);
//		}
//		
//		return mop2fToMOP(newContours2F);
//	}
//	
//	public ArrayList<MatOfPoint> mop2fToMOP(ArrayList<MatOfPoint2f> mats2f){
//		ArrayList<MatOfPoint> mats = new ArrayList<MatOfPoint>();
//		
//		for(int i = 0; i < mats2f.size(); i++) {
//			MatOfPoint temp = new MatOfPoint(mats2f.get(i).toArray());
//			mats.add(i, temp);	
//		}
//		
//		return mats;
//	}
//	
//	public ArrayList<MatOfPoint2f> mopToMOP2F(ArrayList<MatOfPoint> mats){
//		ArrayList<MatOfPoint2f> mats2f = new ArrayList<MatOfPoint2f>();
//		
//		for(int i = 0; i < mats.size(); i++) {
//			MatOfPoint2f temp = new MatOfPoint2f(mats.get(i).toArray());
//			mats2f.add(i, temp);	
//		}
//		
//		return mats2f;
//	}
	
}