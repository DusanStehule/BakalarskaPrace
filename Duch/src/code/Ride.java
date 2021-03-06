package code;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Ride {

	Robot robot;
	RegulatedMotor motorL;
	RegulatedMotor motorR;
	EV3UltrasonicSensor irSensor;
	SampleProvider rangeSampler;
	EV3TouchSensor touchL;
	EV3TouchSensor touchR;
	SensorMode touchLeft;
	SensorMode touchRight;
	boolean done;

	public Ride() {
		robot = new Robot();
		motorL = new EV3LargeRegulatedMotor(MotorPort.A);
		motorR = new EV3LargeRegulatedMotor(MotorPort.B);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		irSensor = new EV3UltrasonicSensor(SensorPort.S1);
		touchL = new EV3TouchSensor(SensorPort.S2);
		touchR = new EV3TouchSensor(SensorPort.S3);
		done = false;
		ride();
		motorL.close();
		motorR.close();
		touchL.close();
		touchR.close();
		irSensor.close();
	}

	public void ride() {
		int help = 0;
		double actual = 0;
		double last = 0;
		double ideal = 0.1;
		double kp = 0.9;
		double kd = 7;
		double action = 0;
		int speed = 500;
		touchLeft = touchL.getTouchMode();
		touchRight = touchR.getTouchMode();
		rangeSampler = irSensor.getDistanceMode();
		float[] sampleL = new float[touchLeft.sampleSize()];
		float[] sampleR = new float[touchRight.sampleSize()];
		float[] lastRange = new float[rangeSampler.sampleSize()];
		rangeSampler.fetchSample(lastRange, 0);
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
			if (help == 1) {
				Delay.msDelay(1000);
			} else {
				Delay.msDelay(500);
			}
			help = 0;

			do {
				touchLeft.fetchSample(sampleL, 0);
				touchRight.fetchSample(sampleR, 0);
				rangeSampler.fetchSample(lastRange, 0);
				last = actual;
				actual = lastRange[0];
				if ((last < 0.18) && (last > 0) && (actual < 0.18) && (actual > 0)) {
					action = kp * (actual - ideal) + kd * (actual - last);
					motorL.startSynchronization();
					motorL.setSpeed((int) (speed * (1 - action)));
					motorR.setSpeed((int) (speed * (1 + action)));
					motorL.endSynchronization();
					touchLeft.fetchSample(sampleL, 0);
					touchRight.fetchSample(sampleR, 0);
					rangeSampler.fetchSample(lastRange, 0);
				} else {
					motorL.startSynchronization();
					motorL.setSpeed(speed);
					motorR.setSpeed(speed);
					motorL.endSynchronization();
					touchLeft.fetchSample(sampleL, 0);
					touchRight.fetchSample(sampleR, 0);
					rangeSampler.fetchSample(lastRange, 0);
				}

			} while (((sampleL[0] == 0) || (sampleR[0] == 0)) && (lastRange[0] < 0.18));

			if ((sampleL[0] != 0) && (sampleR[0] != 0)) {
				robot.stop();
				robot.rideback(5);
				robot.rotationRight();
				System.out.println("tocim doprava");
				help = 0;
			}

			if (lastRange[0] > 0.18) {
				robot.ride(16);
				robot.rotationLeft();
				System.out.println("tocim doleva");
				help = 1;
			}
		}
	}

}
