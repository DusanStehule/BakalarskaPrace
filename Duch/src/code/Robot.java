package code;

import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Robot {

	RegulatedMotor motorL;
	RegulatedMotor motorR;
	EV3TouchSensor touchL;
	EV3TouchSensor touchR;
	EV3UltrasonicSensor irSensor;
	SampleProvider rangeSampler;
	SensorMode touch;
	boolean done;

	public Robot() {

		motorL = new EV3LargeRegulatedMotor(MotorPort.B);
		motorR = new EV3LargeRegulatedMotor(MotorPort.C);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
	//	irSensor = new EV3UltrasonicSensor(SensorPort.S1);
	//	touchL = new EV3TouchSensor(SensorPort.S2);
	//	touchR = new EV3TouchSensor(SensorPort.S3);
		motorL.close();
		motorR.close();
	//	touchL.close();
	//	touchR.close();
	//	irSensor.close();
	}


	public void forward(int speed) {
		motorL.startSynchronization();
		motorL.forward();
		motorR.forward();
		motorL.setSpeed(speed);
		motorR.setSpeed(speed);
		motorL.endSynchronization();
	}
	
	public void backward(int speed) {
		motorL.startSynchronization();
		motorL.backward();
		motorR.backward();
		motorL.setSpeed(speed);
		motorR.setSpeed(speed);
		motorL.endSynchronization();
	}

	public void rotationRight() {
		motorL.startSynchronization();
		motorR.rotate(-165);
		motorL.rotate(165);
		motorL.endSynchronization();
		Delay.msDelay(1000);
	}

	public void rotationLeft() {
		motorL.startSynchronization();
		motorR.rotate(180);
		motorL.rotate(-180);
		motorL.endSynchronization();
		Delay.msDelay(1200);
	}

	public void rotation180() {
		motorL.startSynchronization();
		motorR.rotate(345);
		motorL.rotate(-345);
		motorL.endSynchronization();
		Delay.msDelay(1200);
	}

	public void stop() {
		motorL.startSynchronization();
		motorL.stop();
		motorR.stop();
		motorL.endSynchronization();
	}

	public void ride(int l) {
		double angle = 360 * l / (3.14159 * 5.6);
		motorL.startSynchronization();
		motorL.rotate((int) angle);
		motorR.rotate((int) angle);
		motorL.endSynchronization();
		Delay.msDelay(l * 80);
	}
	
	public void rideback(int l) {
		double angle = 360 * l / (3.14159 * 5.6);
		motorL.startSynchronization();
		motorL.rotate(- (int) angle);
		motorR.rotate(- (int) angle);
		motorL.endSynchronization();
		Delay.msDelay(l * 80);
	}
	

}
