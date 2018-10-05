package ca.mcgill.ecse211.Ultrasonic;

import ca.mcgill.ecse211.lab4.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.robotics.SampleProvider;

public class USLocalizer extends Thread{

	public enum LocalizationType{FALLING_EDGE, RISING_EDGE};
	
	public static final double ROTATION_SPEED = 30.0;
	
	private Odometer odo;
	private Navigation nav;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationType type;
	
	public USLocalizer(Odometer odo2, SampleProvider usValue, float[] usData2) {
		// TODO Auto-generated constructor stub
	}
	
	public void setType(LocalizationType type) {
		this.type = type;
		
	}

}
