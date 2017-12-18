package parkingRobot.hsamr0;

import lejos.geom.Point;
import lejos.robotics.navigation.Pose;

public class CoordinateSystem {
	private Pose pointOfOrigin = null;
	
	public void setPointOfOrigin(Pose poo) {
		pointOfOrigin = poo;
	}

	/**
	 * Compute the Pose of an input in respect to the point of origin of the coordinate system
	 * @param inputPose
	 * @return
	 */
	public Pose getTransformedPose(Pose inputPose) {
		return new Pose((float) (pointOfOrigin.getLocation().getX() - inputPose.getLocation().getX()),
				(float) (pointOfOrigin.getLocation().getY() - inputPose.getLocation().getY()),
				pointOfOrigin.getHeading() - inputPose.getHeading());
	}
	
	/**
	 * Compute the angle of an inputPose in the coordinate System
	 * @param inputPose
	 * @return
	 */
	public float getTransformedHeading(Pose inputPose) {
		return pointOfOrigin.getHeading() - inputPose.getHeading();
	}
	
	public Point getTransformedPoint(Pose inputPose) {
		return pointOfOrigin.getLocation().subtract(inputPose.getLocation());
	}

}
