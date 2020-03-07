package code;
import static code.GripRunner.makeWindow;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static code.GripRunner.makeCamera;

import code.GripRunner;
import code.GripRunner.Listener;
import code.VideoViewer;

import edu.wpi.first.networktables.NetworkTable;

public class Runner {
	
	//320, 240
	static final int IMG_WIDTH = 321 * 2;
	static final int IMG_HEIGHT = 241 * 2;

	final VideoViewer window;
	final Listener<GripPipeline> listener;
	final GripRunner<GripPipeline> gripRunner;
	
	public Runner() {	
		
		this.window = makeWindow("GRIP", IMG_WIDTH, IMG_HEIGHT);
		this.listener = (this.window!=null) ? (processor -> { window.imshow(processor.sourceOutput());}) : null;
		//this.listener = (this.window!=null) ? (processor -> { window.imshow(processor.hsvThresholdOutput());}) : null;
		this.gripRunner = new GripRunner<>(
				makeCamera(0, IMG_WIDTH, IMG_HEIGHT, -1.0), 
				new GripPipeline(), 
				listener);
	}

	public static void main(String[] args) {
		Runner app = new Runner();
		app.gripRunner.runForever();
	}

}

//drawCenters(m_pipeline.findContourCenters(m_pipeline.convexHullsOutput()), m_image);