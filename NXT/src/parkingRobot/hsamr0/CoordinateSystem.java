package parkingRobot.hsamr0;

import lejos.geom.Point;
import lejos.robotics.navigation.Pose;

public class CoordinateSystem {
	private Pose pointOfOrigin = null;

	public void setPointOfOrigin(Pose poo) {
		pointOfOrigin = poo;
	}

	/**
	 * Compute the Pose of an input in respect to the point of origin of the
	 * coordinate system
	 * 
	 * @param inputPose
	 * @return
	 */
	public Pose getTransformedPose(Pose inputPose) {
		return new Pose((float) (inputPose.getLocation().getX() - pointOfOrigin.getLocation().getX()),
				(float) (inputPose.getLocation().getY() - pointOfOrigin.getLocation().getY()),
				inputPose.getHeading() - pointOfOrigin.getHeading());
	}

	/**
	 * Compute the angle of an inputPose in the coordinate System
	 * 
	 * @param inputPose
	 * @return
	 */
	public float getTransformedHeading(Pose inputPose) {
		return inputPose.getHeading() - pointOfOrigin.getHeading();
	}

	public Point getTransformedPoint(Pose inputPose) {
		return getTransformedPoint(inputPose.getLocation());
	}

	public Point getTransformedPoint(Point inputPoint) {
		return inputPoint.subtract(pointOfOrigin.getLocation());
	}
}
