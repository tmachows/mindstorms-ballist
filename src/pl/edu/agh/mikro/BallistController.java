package pl.edu.agh.mikro;

import java.math.RoundingMode;
import java.text.DecimalFormat;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.SensorSelector;

public class BallistController {

	private EV3LargeRegulatedMotor drivingMotor;
	private EV3LargeRegulatedMotor firingMotor;
	private EV3MediumRegulatedMotor liftingMotor;
	private Port colorPort;
	private Port distancePort;
	private SensorModes colorSensor;
	private SensorModes distanceSensor;
	private SampleProvider color;
	private SampleProvider distance;
	private float[] colorSample;
	private float[] distanceSample;
	private static final int MAX_AMMO = 4; 
	

	public BallistController() {
		
		drivingMotor = new EV3LargeRegulatedMotor(MotorPort.C);
		firingMotor = new EV3LargeRegulatedMotor(MotorPort.B);
		liftingMotor = new EV3MediumRegulatedMotor(MotorPort.A);	
		
		colorPort = LocalEV3.get().getPort("S3");
		distancePort = LocalEV3.get().getPort("S4");
		
		colorSensor = new EV3ColorSensor(colorPort);
		distanceSensor = new EV3UltrasonicSensor(distancePort);
		
		color = colorSensor.getMode("ColorID");
		distance = distanceSensor.getMode("Distance");
		
		colorSample = new float[color.sampleSize()];
		distanceSample = new float[distance.sampleSize()];
	}

	public void run() {
		// tymczasowy algorytm do testowania
		
//		for (int i=0; i<3; i++) {
//			drivingMotor.rotate(3*360);
//			drivingMotor.rotate(-90);
//			liftingMotor.rotate(10);
//			firingMotor.rotate(10*360);
//		}
		
		firingMotor.setSpeed(1000);
		firingMotor.rotate(20*360);
		
		this.close();
		
		// algorytm wlasciwy:
		/*
		for (int i=0; i<MAX_AMMO; i++) {
			this.goAndCheckTargets();
			this.aim();
			this.shoot();
		}
		*/
		
		// this.close();
	}

	public void goAndCheckTargets() {
		boolean targetSpotted = false;
		while (!targetSpotted) {
			drivingMotor.rotate(1);
			
			color.fetchSample(colorSample, 0);
			if(colorSample[0] == 1.0) {
				targetSpotted = true;
			}
		}
		distance.fetchSample(distanceSample, 0);
	}
	
	public void aim() {
		liftingMotor.rotate((int) (distanceSample[0] - 0.5) * 90);
	}
	
	public void shoot() {
		firingMotor.setSpeed(1200);
		firingMotor.rotate(10*360);	
	}

	public void close() {
		drivingMotor.close();
		firingMotor.close();
		liftingMotor.close();
	}
}
