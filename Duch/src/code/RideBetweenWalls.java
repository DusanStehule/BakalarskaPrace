package code;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class RideBetweenWalls {

	Robot robot;
	RegulatedMotor motorL;
	RegulatedMotor motorR;
	EV3TouchSensor touch1;
	EV3TouchSensor touch2;
	SensorMode touchLeft;
	SensorMode touchRight;
	EV3UltrasonicSensor distanceSensor;
	SampleProvider distanceSampler;

	public RideBetweenWalls() {
		robot = new Robot();
		motorL = new EV3LargeRegulatedMotor(MotorPort.A);
		motorR = new EV3LargeRegulatedMotor(MotorPort.B);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		touch1 = new EV3TouchSensor(SensorPort.S3);
		distanceSensor = new EV3UltrasonicSensor(SensorPort.S2);
		touch2 = new EV3TouchSensor(SensorPort.S1);
		ride();
		motorL.close();
		motorR.close();
		touch1.close();
		touch2.close();
	}

	public void ride() {
		double actual = 0;
		double last = 0;
		double ideal = 0.07;
		double kp = 0.4;
		double kd = 3.5;
		double action = 0;
		int speed = 500;

		touchLeft = touch1.getTouchMode();
		touchRight = touch2.getTouchMode();
		distanceSampler = distanceSensor.getDistanceMode();
		float[] sampleL = new float[touchLeft.sampleSize()];
		float[] sampleR = new float[touchRight.sampleSize()];
		float[] lastRange = new float[distanceSampler.sampleSize()];
		actual = lastRange[0];
		
		while (true) {
			motorL.startSynchronization();
			motorL.stop();
			motorR.stop();
			motorL.forward();
			motorR.forward();
			motorL.setSpeed(speed);
			motorR.setSpeed(speed);
			motorL.endSynchronization();
			kp = 1;
			kd = 7;

			do {
				touchLeft.fetchSample(sampleL, 0);
				distanceSampler.fetchSample(lastRange, 0);
				last = actual;
				actual = lastRange[0];
				if ((last > 0) && (actual > 0)) {
					action = kp * (actual - ideal) + kd * (actual - last);
					motorL.startSynchronization();
					motorL.setSpeed((int) (speed * (1 - action)));
					motorR.setSpeed((int) (speed * (1 + action)));
					motorL.endSynchronization();
				} else {
					motorL.startSynchronization();
					motorL.setSpeed(speed);
					motorR.setSpeed(speed);
					motorL.endSynchronization();
				}
				touchLeft.fetchSample(sampleL, 0);
			} while (sampleL[0] == 0);

			System.out.println("otacim se");
			
			motorL.startSynchronization();
			motorL.stop();
			motorR.stop();
			motorL.backward();
			motorR.backward();
			motorL.setSpeed(speed);
			motorR.setSpeed(speed);
			motorL.endSynchronization();
			kp = 0;
			kd = 0;
			
			do {
				touchRight.fetchSample(sampleR, 0);
				distanceSampler.fetchSample(lastRange, 0);
				last = actual;
				actual = lastRange[0];
				if ((last > 0) && (actual > 0)) {
					action = kp * (actual - ideal) + kd * (actual - last);
					motorL.startSynchronization();
					motorL.setSpeed((int) (speed * (1 - action)));
					motorR.setSpeed((int) (speed * (1 + action)));
					motorL.endSynchronization();
				} else {
					motorL.startSynchronization();
					motorL.setSpeed(speed);
					motorR.setSpeed(speed);
					motorL.endSynchronization();
				}
				touchRight.fetchSample(sampleR, 0);
			} while (sampleR[0] == 0);

			System.out.println("otacim se");

		}
	}
}
