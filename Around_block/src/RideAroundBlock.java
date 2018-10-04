
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class RideAroundBlock {

	RegulatedMotor motorL;
	RegulatedMotor motorR;
	RegulatedMotor motorSmall;
	
	EV3UltrasonicSensor distanceSensor;
	EV3UltrasonicSensor light;
	SampleProvider distanceSampler;

	public RideAroundBlock() {
		motorL = new EV3LargeRegulatedMotor(MotorPort.B);
		motorR = new EV3LargeRegulatedMotor(MotorPort.C);
		motorSmall = new EV3LargeRegulatedMotor(MotorPort.D);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		distanceSensor = new EV3UltrasonicSensor(SensorPort.S2);
		light = new EV3UltrasonicSensor(SensorPort.S4);
		Button.waitForAnyPress();
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
		int constant = 0;
		distanceSampler = distanceSensor.getDistanceMode();
		float[] lastRange = new float[distanceSampler.sampleSize()];
		distanceSampler.fetchSample(lastRange, 0);
		actual = lastRange[0];
		motorSmall.resetTachoCount();
		

		while (!Button.ESCAPE.isDown()) {
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
				if (constant == 0) {
					motorSmall.setSpeed(80);
					motorSmall.forward();
				} else {
					motorSmall.setSpeed(80);
					motorSmall.backward();
				}
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
				
				
				if (motorSmall.getTachoCount() > 350) {
					constant = 1;
				}
				
				if (motorSmall.getTachoCount() < 10) {
					constant = 0;
				}
				
			} while (lastRange[0] < 0.2);

			ride(12);
			rotationLeft();
			System.out.println("tocim doleva");
			actual = 0;
		}
	}
	
	public void ride(int l) {
		double angle = 360 * l / (3.14159 * 5.6);
		motorL.startSynchronization();
		motorL.rotate((int) angle);
		motorR.rotate((int) angle);
		motorL.endSynchronization();
		Delay.msDelay(l * 80);
	}
	
	public void rotationLeft() {
		motorL.startSynchronization();
		motorR.rotate(185);
		motorL.rotate(-185);
		motorL.endSynchronization();
		Delay.msDelay(1200);
	}
}
