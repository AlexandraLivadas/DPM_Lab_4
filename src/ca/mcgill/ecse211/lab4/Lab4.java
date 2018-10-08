package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.Ultrasonic.USLocalizer;
import ca.mcgill.ecse211.Ultrasonic.USLocalizer.LocalizationType;
import ca.mcgill.ecse211.Ultrasonic.UltrasonicPoller;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab4 {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	private static final Port usPort = LocalEV3.get().getPort("S4");
	private static final Port lsPort = LocalEV3.get().getPort("S1");
	
	//Setting up ultrasonic sensor
	public static SensorModes usSensor = new EV3UltrasonicSensor(usPort);
	public static SampleProvider usValue = usSensor.getMode("Distance");
	public static float[] usData = new float[usValue.sampleSize()];
	//Setting up light sensor
	public static SensorModes lsSensor = new EV3ColorSensor(lsPort);
	public static SampleProvider lsValue = lsSensor.getMode("Red");
	public static float[] lsData = new float[lsValue.sampleSize()];
	
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	
	public static final double WHEEL_RAD = 2.2;
	public static final double WHEEL_BASE = 10.05;
	
	
	public static void main(String[] args) throws OdometerExceptions {
		
		int buttonChoice;

		//Setting up the odometer and display
		Odometer odo = Odometer.getOdometer(leftMotor, rightMotor, WHEEL_BASE, WHEEL_RAD);
		Display display = new Display(lcd);
		USLocalizer USLocal = new USLocalizer(odo, usValue, usData);
		UltrasonicPoller usPoller = new UltrasonicPoller(usSensor, usData, USLocal);
		LightLocalizer lightLocal = new LightLocalizer(odo, lsValue, lsData);
		
		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Falling| Rising ", 0, 2);
			lcd.drawString(" edge  |   edge ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
			
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) { //US Localization has been selected
		      // clear the display
		      lcd.clear();

		      USLocal.setType(LocalizationType.FALLING_EDGE);
		      
		      // Start odometer and display threads
		      Thread odoThread = new Thread(odo);
		      odoThread.start();
		      Thread displayThread = new Thread(display);
		      displayThread.start();
		      Thread usPollerThread = new Thread(usPoller);
		      usPollerThread.start();
		      
		      
		      Thread lightThread = new Thread(lightLocal);
		      lightThread.start();

		} else { 
			lcd.clear();
			
		      // clear the display
		      lcd.clear();

		      USLocal.setType(LocalizationType.RISING_EDGE);
		      
		      // Start odometer and display threads
		      Thread odoThread = new Thread(odo);
		      odoThread.start();
		      Thread displayThread = new Thread(display);
		      displayThread.start();
		      Thread usPollerThread = new Thread(usPoller);
		      usPollerThread.start();
		      Thread lightThread = new Thread(lightLocal);
		      lightThread.start();

			
		}
		

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	
		
	}
}
