package code;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Listen {

	EV3UltrasonicSensor listenSensor;
	EV3UltrasonicSensor distanceSensor;
	SampleProvider listenSampler;
	SampleProvider distanceSampler;

	public Listen() {
		listenSensor = new EV3UltrasonicSensor(SensorPort.S2);
		
		distanceSampler = listenSensor.getDistanceMode();
		loop();
	}

	public void loop() {
		while (true) {
			listenSampler = listenSensor.getListenMode();
			float[] lRange = new float[listenSampler.sampleSize()];
			listenSampler.fetchSample(lRange, 0);
			double actual = lRange[0];
			System.out.println("listen" + actual);
			Delay.msDelay(1000);
			
			listenSampler = listenSensor.getListenMode();
			float[] dRange = new float[distanceSampler.sampleSize()];
			distanceSampler.fetchSample(dRange, 0);
			double actual1 = dRange[0];
			System.out.println("distance" + actual1);
			Delay.msDelay(1000);
		}
	}

}
