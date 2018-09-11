package code;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class Transmitter {

	EV3UltrasonicSensor distanceSensor;
	SampleProvider distanceSampler;

	public Transmitter() {

		distanceSensor = new EV3UltrasonicSensor(SensorPort.S2);
		ride();
	}

	public void ride() {
	
		distanceSampler = distanceSensor.getDistanceMode();
		float[] lastRange = new float[distanceSampler.sampleSize()];
		
		while (true) {
				distanceSampler.fetchSample(lastRange, 0);
		}
	}
}

