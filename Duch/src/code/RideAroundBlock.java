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

public class RideAroundBlock {

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

	public RideAroundBlock() {
		robot = new Robot();
		motorL = new EV3LargeRegulatedMotor(MotorPort.A);
		motorR = new EV3LargeRegulatedMotor(MotorPort.B);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		touchL = new EV3TouchSensor(SensorPort.S2);
		irSensor = new EV3UltrasonicSensor(SensorPort.S1);
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
		double actual = 0;
		double last1 = 0;
		double last2 = 0;
		double ideal = 0.05;
		double kp = 1;
		double kd = 5;
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
			Delay.msDelay(100);

			do {
				touchLeft.fetchSample(sampleL, 0);
				touchRight.fetchSample(sampleR, 0);
				rangeSampler.fetchSample(lastRange, 0);
				last1 = actual;
				last2 = last1;
				actual = lastRange[0];
				if ((last1 < 0.2) && (last1 > 0) && (last2 < 0.2) && (last2 > 0) && (actual < 0.2) && (actual > 0)) {
					action = kp * (last1 - ideal) + kd * (last1 - last2);
					motorL.startSynchronization();
					motorL.setSpeed((int) (speed * (1 - action)));
					motorR.setSpeed((int) (speed * (1 + action)));
					motorL.endSynchronization();
					Delay.msDelay(100);
				} else {
					motorL.startSynchronization();
					motorL.setSpeed(speed);
					motorR.setSpeed(speed);
					motorL.endSynchronization();
				}

			} while (((sampleL[0] == 0) || (sampleR[0] == 0)) && (lastRange[0] < 0.2));

			if ((sampleL[0] != 0) && (sampleR[0] != 0)) {
				robot.stop();
				motorL.startSynchronization();
				motorL.rotateTo(-10);
				motorR.rotateTo(-10);
				motorL.endSynchronization();
				Delay.msDelay(200);
				robot.rotationRight();
				System.out.println("tocim doprava");
			}

			if (lastRange[0] > 0.2) {
				robot.ride(12);
				robot.rotationLeft();
				robot.ride(17);
				System.out.println("tocim doleva");
			}
		}
	}

}
