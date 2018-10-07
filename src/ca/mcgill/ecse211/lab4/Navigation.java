package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread{

	private Odometer odo;
	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static final int ACCEL = 300;
	
	private final double WHEEL_RAD = 2.2;
	private final double WHEEL_BASE = 10.0;
	private final double TILE_SIZE = 30.48;
	private double x, y, theta;
	private boolean isNavigating;
	
	public Navigation(Odometer odo) {
		this.odo = odo;
		this.leftMotor = odo.leftMotor;
		this.rightMotor = odo.rightMotor;
	}
	
	void travelTo(double navX, double navY) {

//		UltrasonicPoller usPoller = null;
//		if (UltrasonicPoller.getInstance() != null) {
//			usPoller = UltrasonicPoller.getInstance();
//		}

		// get current coordinates
		theta = odo.getXYT()[2];
		x = odo.getXYT()[0];
		y = odo.getXYT()[1];	


		double deltaX = navX - x;
		double deltaY = navY - y;

		// get absolute values of deltas
		double absDeltaX = Math.abs(deltaX);
		double absDeltaY = Math.abs(deltaY);

		//need to convert theta from degrees to radians
		double deltaTheta = Math.atan2(deltaX, deltaY) / Math.PI * 180;

		// turn to the correct direction
		this.turnTo(theta, deltaTheta);
		Sound.beep();
		
		// move until destination is reached
		// while loop is used in case of collision override
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();

		while(true) {
			double newTheta, newX, newY;
			
			double xyt[] = odo.getXYT();
			//need to convert theta from degrees to radians
			newTheta = xyt[2];
			newX = xyt[0];
			newY = xyt[1];	

			//If the difference between the current x/y and the x/y we started from is similar to the deltaX/deltaY, 
			//Stop the motors because the point has been reached
			if (Math.pow(newX - x, 2) + Math.pow(newY - y, 2) > Math.pow(absDeltaX, 2) + Math.pow(absDeltaY, 2)) {
				break;
			}

			// This is only relevant if the ultrasonic poller thread is being used
//			if (usPoller != null) {
//				if (usPoller.isInitializing) { 	//isInitializing is true when the distance is too close
//					leftMotor.stop(true);
//					rightMotor.stop(false);
//					usPoller.init(navX, navY); //hopefully blocking
//					
//					try {
//						synchronized(usPoller.doneAvoiding) {
//							while(usPoller.isAvoiding) {
//								usPoller.doneAvoiding.wait();
//							}
//							Sound.beepSequenceUp();
//						}
//					} catch(InterruptedException e) {
//						e.printStackTrace();
//					}
//					
//					this._coordsList.add(0, new double[] {navX, navY});
//					
//					return false;
//				}
//			}

			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		}
		leftMotor.stop(true);
		rightMotor.stop(false);
		this.isNavigating = false;
		Sound.twoBeeps();
	}
	
	void turnTo(double currTheta, double destTheta) {
		// get theta difference
		double deltaTheta = destTheta - currTheta;
		// normalize theta (get minimum value)
		deltaTheta = normalizeAngle(deltaTheta);

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		leftMotor.rotate(convertAngle(WHEEL_RAD, WHEEL_BASE, deltaTheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, WHEEL_BASE, deltaTheta), false);
	}
	
	//Getting the minimum angle to turn:
	//It is easier to turn +90 than -270
	//Also, it is easier to turn -90 than +270
	double normalizeAngle(double theta) {
		if (theta <= -180) {
			theta += 360;
		}
		else if (theta > 180) {
			theta -= 360;
		}
		return theta;
	}
	
	public int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}

