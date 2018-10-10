package ca.mcgill.ecse211.lab4;

import java.text.DecimalFormat;

import ca.mcgill.ecse211.Ultrasonic.USLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
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
	private double lightThreshold = 20.0;
	public float lightSensorIntensity;
	private double sensorDistance = 11.3; //in cm, 4.5inches
	private final double WHEEL_RAD = 2.2;
	private double[] lineAngles, linePos;
	private int TURN_ERROR = 5;

	
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 100;
	
	
	public LightLocalizer(Odometer odo, SampleProvider ls, float[] lsData) {
		this.odo = odo;
		this.ls = ls;
		this.lsData = lsData;
		this.nav = new Navigation(odo);
	}
	
	public void run() {
		
		synchronized(USLocalizer.done) {
			try {
				USLocalizer.done.wait();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
		long correctionStart, correctionEnd;
		float intensity;
		firstReading = -1;

		
		//At this point, the robot will (ideally) be at 0-degrees
		//Turning it 45 degrees will (ideally) turn it in the direction of (0,0)
		//***Might want to change this to just 45???
		nav.turnTo(odo.getXYT()[2], 45);
		
		isNavigating = true;
		
		odo.leftMotor.setSpeed(ROTATE_SPEED);
		odo.rightMotor.setSpeed(ROTATE_SPEED);
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
	    	  			odo.leftMotor.stop(true);
	    	  			odo.rightMotor.stop(false);
	    	  			isNavigating = false;
	    	  		}
	      } 
		}
		
		odo.leftMotor.rotate(-nav.convertDistance(WHEEL_RAD , sensorDistance), true);
		odo.rightMotor.rotate(-nav.convertDistance(WHEEL_RAD, sensorDistance), false);
		
		// make counter clock-wise motion so set the x axis first
		
//		odo.leftMotor.rotate(nav.convertAngle(odo.WHEEL_RAD, odo.TRACK, -90), true);
//		odo.rightMotor.rotate(-nav.convertAngle(odo.WHEEL_RAD, odo.TRACK, -90), false);
		
		while(isNavigating) {
		      correctionStart = System.currentTimeMillis();
		      ls.fetchSample(lsData, 0); // acquire data from sensor
		      intensity = (float) (lsData[0] * 100.0); // extract from buffer, cast to float
		      this.lightSensorIntensity = intensity;
		      /*If the current reading is significantly less than the first reading 
		       * (aka 30% less), a line is being passed. 
		       * Has to be a significant enough change, since the panels are not a uniform color. */
		      if ((100*Math.abs(intensity - firstReading)/firstReading) > lightThreshold) { 
		    	  		if (intensity < firstReading) {
		    	  			odo.leftMotor.stop(true);
		    	  			odo.rightMotor.stop(false);
		    	  			isNavigating = false;
		    	  		}
		      } 
			}
		
		//odo.leftMotor.rotate(-nav.convertDistance(WHEEL_RAD , sensorDistance - 3), true);
		//odo.rightMotor.rotate(-nav.convertDistance(WHEEL_RAD, sensorDistance - 3), false);
		
		// start rotating to find the lines
		odo.leftMotor.backward();
		odo.rightMotor.forward();
		
		lineCount = 0;
		lineAngles = new double[4];
		linePos = new double[3];
		
		while (lineCount < 4) {
			ls.fetchSample(lsData, 0); // acquire data from sensor
		    intensity = (float) (lsData[0] * 100.0); // extract from buffer, cast to float
		    
		    if ((100*Math.abs(intensity - firstReading)/firstReading) > lightThreshold) {
		    		
		    		if(lineCount == 3) {
		    			odo.leftMotor.stop(true);
		    			odo.rightMotor.stop(false);
		    		}
		    		linePos = odo.getXYT();
		    		
		    		lineAngles[lineCount] = linePos[2];
		    		
		    		Sound.beep();
		    		
		    		lineCount++;
		    			
		    }
		    
	    		try {
	    			Thread.sleep(20);
	    		} catch (InterruptedException e) {
	    			
	    		}
//		    odo.leftMotor.setSpeed(50);
//		    odo.rightMotor.setSpeed(50);
//		    odo.leftMotor.backward();
//		    odo.rightMotor.forward();
		    
		}
		

		
		//Trigonometry calculations from tutorial
		//assuming that we start in bottom left square
		thetaY = Math.abs(lineAngles[2] - lineAngles[0]);
		thetaX =  Math.abs(lineAngles[3] - lineAngles[1]);
		
		correctedX = -sensorDistance*Math.cos(Math.toRadians(thetaY/2));
		correctedY = -sensorDistance*Math.cos(Math.toRadians(thetaX/2));
		
		//nav.turnTo(odo.getXYT()[2], 0);
		//odo.setPosition(new double [] {correctedX, correctedY, 0.0}, new boolean [] {true, true, true});	

		//Not sure about deltaThetaX calc. ***Could be lineAngles[2]
		
		//current theta value
		deltaThetaX = (270 + (thetaX/2) - lineAngles[3]) % 360;
		deltaThetaY = (540 + (thetaY/2) - lineAngles[0]) % 360;
		
		deltaTheta = (deltaThetaX + deltaThetaY)/2.0;
//		
//		correctedTheta = odo.getXYT()[2] - deltaTheta;
//		correctedTheta = nav.normalizeAngle(correctedTheta);
		
//		odo.setX(correctedX);
//		odo.setY(correctedY);

		odo.setXYT(correctedX, correctedY, -deltaTheta);	

		nav.travelTo(0, 0);
		nav.turnTo(odo.getXYT()[2], 0);
		//nav.turnTo(odo.getXYT()[2], 360-TURN_ERROR);
//		odo.setTheta(0);
		
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
			
		}

	}

	// method moves the robot forwards and updates the isNavigating value
	private void forward()
	{
		odo.leftMotor.forward();
		odo.rightMotor.forward();
		isNavigating = true;
	}
}

