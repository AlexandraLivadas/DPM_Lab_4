package ca.mcgill.ecse211.Ultrasonic;

import ca.mcgill.ecse211.lab4.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.OdometryPoseProvider;

public class USLocalizer extends Thread{

	public enum LocalizationType{FALLING_EDGE, RISING_EDGE};
	
	public static final double ROTATION_SPEED = 30.0;
	
	private Odometer odo;
	private Navigation nav;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationType type;
	private int filterControl;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private float prevDistance;
	private double thetaA, thetaB, thetaAv, thetaZero;
	private float distance;

	public int FILTER_OUT = 3;
	public static int MAX_DIST = 50;
	public static int WALL_DIST = 30;
	public static int WALL_ERROR = 3;
	public static int TURN_ERROR = 8;
	
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	public static int ACCEL = 300;
	
	public USLocalizer(Odometer odo, SampleProvider usSensor, float[] usData) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.filterControl = 0;
		this.leftMotor = odo.leftMotor;
		this.rightMotor = odo.rightMotor;
		prevDistance = 50;
		this.nav = new Navigation(odo);
		
	}
	
	public void setType(LocalizationType type) {
		this.type = type;
		
	}
	
	public void run() {
		double[] pos = new double[3];
		double thetaA, thetaB;
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.setAcceleration(ACCEL);
		rightMotor.setAcceleration(ACCEL);
		

	    if (type == LocalizationType.FALLING_EDGE) {
	    		//Robot will rotate clockwise until there is no wall in front of it
	    		while(processData() < WALL_DIST + WALL_ERROR) {
	    			leftMotor.forward();
	    			rightMotor.backward();
	    		}
	    		//Robot continues to rotate clockwise until it sees another wall in front of it
	    		while(processData() > WALL_DIST) {
	    			leftMotor.forward();
	    			rightMotor.backward();
	    		}
	    		//The angle from the odometer is stored, motors are stopped
	    		thetaA = odo.getXYT()[2];
	    		leftMotor.stop();
	    		rightMotor.stop();
	    		
	    		//Robot rotates counterclockwise until there is no wall in front of it
	    		while(processData() < WALL_DIST + WALL_ERROR) {
	    			leftMotor.backward();
	    			rightMotor.forward();
	    		}
	    		//Robot continues to rotate counterclockwise until it sees anotherwall in front of it
	    		while(processData() > WALL_DIST) {
	    			leftMotor.backward();
	    			rightMotor.forward();
	    		}
	    		//Angle from the odometer is stored, motors are stopped
	    		thetaB = odo.getXYT()[2];
	    		leftMotor.stop();
	    		rightMotor.stop();
	    		
	    		//Need to make sure thetaA isn't greater than 360
	    		if (thetaA > thetaB) {
	    			thetaA -= 360;
	    		}
	    		
	    		//thetaAv will be 45 degrees away from zero degrees
	    		thetaAv = (thetaA + thetaB)/2.0;
	    		thetaZero = thetaB - thetaAv + 45;
	    		
	    		leftMotor.rotate(convertAngle(odo.WHEEL_RAD, odo.TRACK, thetaZero + TURN_ERROR), true);
	    		rightMotor.rotate(-convertAngle(odo.WHEEL_RAD, odo.TRACK, thetaZero + TURN_ERROR), false);
	    		
	    		odo.setPosition(new double[] {0.0, 0.0, 0.0}, new boolean[] {true, true, true});
	    		
	    } else {
	    		//Robot will rotate counterclockwise
	    		while (processData() > WALL_DIST - WALL_ERROR) {
	    			leftMotor.backward();
	    			rightMotor.forward();
	    		}
	    		while (processData() < WALL_DIST) {
	    			leftMotor.backward();
	    			rightMotor.forward();
	    		}
	    		thetaA = odo.getXYT()[2];
    			leftMotor.stop();
    			rightMotor.stop();
    			
    			while(processData() > WALL_DIST - WALL_ERROR) {
	    			leftMotor.forward();
	    			rightMotor.backward();
    			}
    			while(processData() < WALL_DIST) {
	    			leftMotor.forward();
	    			rightMotor.backward();
    			}
    			thetaB = odo.getXYT()[2];
    			leftMotor.stop();
    			rightMotor.stop();
    			
    			if (thetaA > thetaB) {
    				thetaA -= 360;
    			}
    			thetaAv = (thetaA + thetaB)/2.0;
    			thetaZero = thetaB - thetaAv + 45;
    			
    			leftMotor.rotate(convertAngle(odo.WHEEL_RAD, odo.TRACK, thetaZero - TURN_ERROR), true);
	    		rightMotor.rotate(-convertAngle(odo.WHEEL_RAD, odo.TRACK, thetaZero - TURN_ERROR), false);
	    		
	    		odo.setPosition(new double[] {0.0, 0.0, 0.0}, new boolean[] {true, true, true});
	    }
		
	}
	
	private  int convertDistance(double radius, double distance)
	{ 															 
		return (int) ((180.0 * distance) / (Math.PI * radius)); 
	} 
	      
	private  int convertAngle(double radius, double width, double angle) 
	{ 
		return convertDistance(radius, Math.PI * width * angle / 360.0); 
	}
	
	private float processData() {
		usSensor.fetchSample(usData,  0);
		float distance = (int)(100.0 * usData[0]);
		float result = 0;
		if (distance >= 255 && filterControl < FILTER_OUT) {
			filterControl++;
			result = prevDistance;
		} else if (distance >= 255) {
			result = MAX_DIST;
		} else {
			filterControl = 0;
			result = distance;
		}
		prevDistance = distance;
		return result;
	}
	
}
