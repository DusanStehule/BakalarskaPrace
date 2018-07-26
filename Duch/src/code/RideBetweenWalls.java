package code;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;

public class RideBetweenWalls {

	RegulatedMotor motor;
	EV3TouchSensor touchL;
	EV3TouchSensor touchR;
	SensorMode touchLeft;
	SensorMode touchRight;

	public RideBetweenWalls() {
		motor = new EV3LargeRegulatedMotor(MotorPort.A);
		touchL = new EV3TouchSensor(SensorPort.S1);
		touchR = new EV3TouchSensor(SensorPort.S2);
		ride();
		motor.close();
		touchL.close();
		touchR.close();
	}

	public void ride() {
		int speed = 500;

		touchLeft = touchL.getTouchMode();
		touchRight = touchR.getTouchMode();
		float[] sampleL = new float[touchLeft.sampleSize()];
		float[] sampleR = new float[touchRight.sampleSize()];

		while (true) {
			motor.forward();
			motor.setSpeed(speed);

			do {
				touchLeft.fetchSample(sampleL, 0);
				touchRight.fetchSample(sampleR, 0);
			} while (sampleL[0] == 0);

			System.out.println("otacim se");
			
			motor.stop();
			motor.backward();
			motor.setSpeed(speed);
			
			do {
				touchLeft.fetchSample(sampleL, 0);
				touchRight.fetchSample(sampleR, 0);
			} while (sampleR[0] == 0);

			System.out.println("otacim se");

		}
	}
}
