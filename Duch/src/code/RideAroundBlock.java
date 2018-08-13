package code;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class RideAroundBlock {

	Robot robot;
	RegulatedMotor motorL;
	RegulatedMotor motorR;
	EV3UltrasonicSensor distanceSensor;
	SampleProvider distanceSampler;

	public RideAroundBlock() {
		robot = new Robot();
		motorL = new EV3LargeRegulatedMotor(MotorPort.A);
		motorR = new EV3LargeRegulatedMotor(MotorPort.B);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		distanceSensor = new EV3UltrasonicSensor(SensorPort.S1);
		ride();
		motorL.close();
		motorR.close();
		distanceSensor.close();
	}

	public void ride() {
		double actual = 0;
		double last = 0;
		double ideal = 0.07;
		double kp = 0.9;
		double kd = 7;
		double action = 0;
		int speed = 500;
		distanceSampler = distanceSensor.getDistanceMode();
		float[] lastRange = new float[distanceSampler.sampleSize()];
		distanceSampler.fetchSample(lastRange, 0);
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
			Delay.msDelay(1000);

			do {
				distanceSampler.fetchSample(lastRange, 0);
				last = actual;
				actual = lastRange[0];
				if ((last < 0.2) && (last > 0) && (actual < 0.2) && (actual > 0)) {
					action = kp * (actual - ideal) + kd * (actual - last);
					motorL.startSynchronization();
					motorL.setSpeed((int) (speed * (1 - action)));
					motorR.setSpeed((int) (speed * (1 + action)));
					motorL.endSynchronization();
					distanceSampler.fetchSample(lastRange, 0);
				} else {
					motorL.startSynchronization();
					motorL.setSpeed(speed);
					motorR.setSpeed(speed);
					motorL.endSynchronization();
					distanceSampler.fetchSample(lastRange, 0);
				}
			} while (lastRange[0] < 0.2);

			robot.ride(12);
			robot.rotationLeft();
			System.out.println("tocim doleva");
			actual = 0;
		}
	}
}
