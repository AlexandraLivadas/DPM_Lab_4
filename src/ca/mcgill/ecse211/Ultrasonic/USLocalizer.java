package ca.mcgill.ecse211.Ultrasonic;

import ca.mcgill.ecse211.lab4.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.OdometryPoseProvider;

public class USLocalizer extends Thread{

	public enum LocalizationType{FALLING_EDGE, RISING_EDGE};

	/**
	 * 
	 * RESET state ensure robot isn't within the raising or falling edge limit
	 * SEEK state seeks thetaA
	 * SEEK_2 state seeks thetaB
	 * CORRECTION state corrects robot direction
	 *
	 */
	public enum LocalizationState{RESET, SEEK, SEEK_2, CORRECTION, DONE};


	public static final double ROTATION_SPEED = 30.0;

	
	public static Object done = new Object();
	
	
	private Odometer odo;
	private Navigation nav;
	private LocalizationType type;
	private LocalizationState state;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private int prevDistance;
	private double thetaA, thetaB, thetaAv, thetaZero;
	private int filterControl;
	
	
	public int FILTER_OUT = 20;
	public static int FILTER_OUT_DIST = 50;
	public static int WALL_DIST = 40;
	public static int WALL_ERROR = 8;
	public static int TURN_ERROR = 0;

	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 120;
	public static int ACCEL = 300;

	public USLocalizer(Odometer odo, SampleProvider usSensor, float[] usData) {
		this.odo = odo;

		this.leftMotor = odo.leftMotor;
		this.rightMotor = odo.rightMotor;
		prevDistance = 50;
		this.nav = new Navigation(odo);
		this.state = LocalizationState.RESET;
		
		this.filterControl = 0;
		this.prevDistance = Integer.MAX_VALUE;
		
		
		// initialize motor speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
//		leftMotor.setAcceleration(ACCEL);
//		rightMotor.setAcceleration(ACCEL);

	}

	public void setType(LocalizationType type) {
		this.type = type;

	}	

	public void processUSData(int distance) {

		
		// filter for the crease of the wall
		if (distance >= 255 && filterControl < FILTER_OUT && prevDistance < distance) {
			filterControl++;
			distance = prevDistance;
		} else if (distance >= 255) {
			// do nothing
		} else {
			filterControl = 0;
		}
		
		if (type == LocalizationType.FALLING_EDGE) {
			// Robot will rotate clockwise until there is no wall in front of it
			// and further more to ensure it goes to falling edge

			switch(state) {
				case RESET:
					if (distance >= FILTER_OUT_DIST) {
						leftMotor.stop(true);
						rightMotor.stop(false);
						odo.setTheta(0);
						Sound.twoBeeps();
						this.state = LocalizationState.SEEK;
					} else {
						leftMotor.backward();
						rightMotor.forward();
					}
					break;
				case SEEK:
					if ((distance <= WALL_DIST + WALL_ERROR) && (prevDistance > WALL_DIST + WALL_ERROR))  {
						leftMotor.stop(true);
						rightMotor.stop(false);
						thetaA = odo.getXYT()[2];
						Sound.beep();
						leftMotor.rotate(convertAngle(odo.WHEEL_RAD, odo.TRACK,-thetaA), true);
						rightMotor.rotate(-convertAngle(odo.WHEEL_RAD, odo.TRACK, -thetaA), false);
						this.state = LocalizationState.SEEK_2;
					} else {
						leftMotor.forward();
						rightMotor.backward();
					}
					break;
				case SEEK_2:
					if ((distance <= WALL_DIST + WALL_ERROR) && (prevDistance > WALL_DIST + WALL_ERROR)) {
						leftMotor.stop(true);
						rightMotor.stop(false);
						thetaB = odo.getXYT()[2] - 360;
						Sound.beep();
						this.state = LocalizationState.CORRECTION;
					} else {
						leftMotor.backward();
						rightMotor.forward();
					}
					break;
				case CORRECTION:
					//thetaAv will be 45 degrees away from zero degrees
					this.thetaAv = (this.thetaA - this.thetaB)/2.0;
					this.thetaZero = thetaAv - 45.0;

					leftMotor.rotate(convertAngle(odo.WHEEL_RAD, odo.TRACK, thetaZero + TURN_ERROR), true);
					rightMotor.rotate(-convertAngle(odo.WHEEL_RAD, odo.TRACK, thetaZero + TURN_ERROR), false);

					odo.setTheta(0);
					this.state = LocalizationState.DONE;
					while (Button.waitForAnyPress() != Button.ID_ENTER);
					synchronized(USLocalizer.done) {
						USLocalizer.done.notifyAll();
					}
					break;	
				default:
					break;
			}
		} else if (type == LocalizationType.RISING_EDGE) {
			switch(state) {
				case RESET:
					if (distance < WALL_DIST - WALL_ERROR) {
						leftMotor.stop(true);
						rightMotor.stop(false);
						odo.setTheta(0);
						Sound.twoBeeps();
						this.state = LocalizationState.SEEK;
					} else {
						leftMotor.backward();
						rightMotor.forward();
					}
					break;
				case SEEK:
					if ((distance >= WALL_DIST - WALL_ERROR) && (prevDistance < WALL_DIST + WALL_ERROR))  {
						leftMotor.stop(true);
						rightMotor.stop(false);
						thetaA = odo.getXYT()[2];
						Sound.beep();
						leftMotor.rotate(convertAngle(odo.WHEEL_RAD, odo.TRACK,-thetaA), true);
						rightMotor.rotate(-convertAngle(odo.WHEEL_RAD, odo.TRACK, -thetaA), false);
						this.state = LocalizationState.SEEK_2;
					} else {
						leftMotor.forward();
						rightMotor.backward();
					}
					break;
				case SEEK_2:
					if ((distance >= WALL_DIST - WALL_ERROR) && (prevDistance < WALL_DIST + WALL_ERROR))  {
						leftMotor.stop(true);
						rightMotor.stop(false);
						thetaB = odo.getXYT()[2] - 360;
						Sound.beep();
						this.state = LocalizationState.CORRECTION;
					} else {
						leftMotor.backward();
						rightMotor.forward();
					}
					break;
				case CORRECTION:
					//thetaAv will be 45 degrees away from zero degrees
					this.thetaAv = (this.thetaA - this.thetaB)/2.0;
					this.thetaZero = this.thetaAv + 135;
	
					leftMotor.rotate(convertAngle(odo.WHEEL_RAD, odo.TRACK, thetaZero + TURN_ERROR), true);
					rightMotor.rotate(-convertAngle(odo.WHEEL_RAD, odo.TRACK, thetaZero + TURN_ERROR), false);
	
					odo.setTheta(0);
					this.state = LocalizationState.DONE;
					while (Button.waitForAnyPress() != Button.ID_ENTER);
					synchronized(USLocalizer.done) {
						USLocalizer.done.notifyAll();
					}
					break;
				default:
					break;
				
			}
		}
		
		// set previous distance
		// this is called before any seeks in the reset state to ensure
		// that this prevDistance will be initialized
		this.prevDistance = distance;

	}

	private  int convertDistance(double radius, double distance)
	{ 															 
		return (int) ((180.0 * distance) / (Math.PI * radius)); 
	} 

	private  int convertAngle(double radius, double width, double angle) 
	{ 
		return convertDistance(radius, Math.PI * width * angle / 360.0); 
	}


}
