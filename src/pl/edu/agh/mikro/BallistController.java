package pl.edu.agh.mikro;

import java.io.File;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

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
	private int aimingRotation = 0;

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
		Sound.setVolume(Sound.VOL_MAX);
		Sound.playSample(new File("army_wake_up.wav"));

		for (int i = 0; i < MAX_AMMO; i++) {
			this.goAndCheckTargets();
			this.aim();
			this.shoot();
		}

		this.close();
	}

	public void goAndCheckTargets() {
		System.out.println("Looking for targets...");
		boolean targetSpotted = false;
		while (!targetSpotted) {
			drivingMotor.setSpeed(180);
			drivingMotor.rotate(-30, true);

			color.fetchSample(colorSample, 0);
			if (colorSample[0] == 1.0) {
				targetSpotted = true;
				System.out.println("Target spotted!");
				drivingMotor.setSpeed(180);
				drivingMotor.rotate(180);
			}
		}
		distance.fetchSample(distanceSample, 0);
		System.out.println("Target distance: " + distanceSample[0]);
	}

	public void aim() {
		// wysokosc kuszy = 17 cm
		liftingMotor.setSpeed(90);
		if (distanceSample[0] < 0.2)
			aimingRotation = 0;
		else if (distanceSample[0] < 0.35)
			aimingRotation = 1;
		else
			aimingRotation = 2;
		liftingMotor.rotate(-aimingRotation * 360);
	}

	public void shoot() {
		System.out.println("Fire!!!");
		firingMotor.setSpeed(1200);
		firingMotor.rotate(25 * 360);
		System.out.println("Target should be down");
		goForNextTarget();
	}

	private void goForNextTarget() {
		liftingMotor.setSpeed(90);
		liftingMotor.rotate(aimingRotation * 360);
		aimingRotation = 0;
		System.out.println("Let's look for another target");
		drivingMotor.setSpeed(180);
		drivingMotor.rotate(-1 * 360);
	}

	public void close() {
		drivingMotor.close();
		firingMotor.close();
		liftingMotor.close();
	}
}
