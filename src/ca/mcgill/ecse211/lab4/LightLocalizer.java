package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class LightLocalizer extends Thread{
	
	private Odometer odo;
	private SampleProvider ls;
	private float[] lsData;
	private Navigation nav;
	
	private int lineCount;
	private double thetaX, thetaY;
	private double correctedX, correctedY, correctedTheta;
	private double deltaThetaX, deltaThetaY, deltaTheta;
	private boolean isNavigating;
	private float firstReading;
	private double lightThreshold = 30.0;
	public float lightSensorIntensity;
	private double sensorDistance = 0;
	private final double WHEEL_RAD = 2.2;
	private double[] lineAngles, linePos;

	
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	
	
	public LightLocalizer(Odometer odo, SampleProvider ls, float[] lsData) {
		this.odo = odo;
		this.ls = ls;
		this.lsData = lsData;
		this.nav = new Navigation(odo);
	}
	
	public void run() {
		long correctionStart, correctionEnd;
		float intensity;
		firstReading = -1;

		
		//At this point, the robot will (ideally) be at 0-degrees
		//Turning it 45 degrees will (ideally) turn it in the direction of (0,0)
		//***Might want to change this to just 45???
		nav.turnTo(odo.getXYT()[2], odo.getXYT()[2]+45);
		
		isNavigating = true;
		
		odo.leftMotor.setSpeed(FORWARD_SPEED);
		odo.rightMotor.setSpeed(FORWARD_SPEED);
		odo.leftMotor.forward();
		odo.rightMotor.forward();

		while(isNavigating) {
	      correctionStart = System.currentTimeMillis();
	      ls.fetchSample(lsData, 0); // acquire data from sensor
	      intensity = (float) (lsData[0] * 100.0); // extract from buffer, cast to float
	      this.lightSensorIntensity = intensity;
	      if (firstReading == -1) { //Set the first reading value
	    	  	firstReading = intensity;
	      }
	      /*If the current reading is significantly less than the first reading 
	       * (aka 30% less), a line is being passed. 
	       * Has to be a significant enough change, since the panels are not a uniform color. */
	      else if ((100*Math.abs(intensity - firstReading)/firstReading) > lightThreshold) { 
	    	  		if (intensity < firstReading) {
	    	  			odo.leftMotor.stop();
	    	  			odo.rightMotor.stop();
	    	  			isNavigating = false;
	    	  		}
	      } 
		}
		
		odo.leftMotor.rotate(nav.convertDistance(WHEEL_RAD , sensorDistance), true);
		odo.rightMotor.rotate(nav.convertDistance(WHEEL_RAD, sensorDistance), false);
		
		lineCount = 0;
		lineAngles = new double[4];
		linePos = new double[3];
		
		while (lineCount < 4) {
			ls.fetchSample(lsData, 0); // acquire data from sensor
		    intensity = (float) (lsData[0] * 100.0); // extract from buffer, cast to float
		    
		    if ((100*Math.abs(intensity - firstReading)/firstReading) > lightThreshold) {
		    		
		    		linePos = odo.getXYT();
		    		
		    		lineAngles[lineCount] = linePos[2];
		    		
		    		Sound.beep();
		    		
		    		lineCount++;
		
		    		try {
		    			Thread.sleep(50);
		    		} catch (InterruptedException e) {
		    			
		    		}
		    }
		   
		    odo.leftMotor.setSpeed(ROTATE_SPEED);
		    odo.rightMotor.setSpeed(ROTATE_SPEED);
		    odo.leftMotor.backward();
		    odo.rightMotor.forward();
		    
		}
		
		odo.leftMotor.stop();
		odo.rightMotor.stop();
		
		//Trigonometry calculations from tutorial
		thetaX = lineAngles[2] - lineAngles[0];
		thetaY = lineAngles[3] - lineAngles[1];
		
		correctedX = sensorDistance*Math.cos(Math.toRadians(thetaY/2));
		correctedY = sensorDistance*Math.cos(Math.toRadians(thetaX/2));
		
		//Not sure about deltaThetaX calc. ***Could be lineAngles[2]
		deltaThetaX = 270 - (thetaX/2) - lineAngles[0];
		deltaThetaY = 270 - (thetaY/2) - lineAngles[3];
		
		deltaTheta = (deltaThetaX + deltaThetaY)/2;
		
		correctedTheta = odo.getXYT()[2] - deltaTheta;
		correctedTheta = nav.normalizeAngle(correctedTheta);
		
		odo.setXYT(correctedX, correctedY, correctedTheta);	
		
		nav.travelTo(0, 0);
		
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
			
		}
		
		nav.turnTo(odo.getXYT()[2], 0);
	}

}

