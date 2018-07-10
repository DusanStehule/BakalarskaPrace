package code;


import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class Robot {

	RegulatedMotor motorL;
	RegulatedMotor motorR;
	EV3TouchSensor touchL;
	EV3TouchSensor touchR;
	SensorMode touch;
	boolean done;
	

	public Robot() {

		motorL = new EV3LargeRegulatedMotor(MotorPort.A);
		motorR = new EV3LargeRegulatedMotor(MotorPort.B);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		touchL = new EV3TouchSensor(SensorPort.S1);
		touchR = new EV3TouchSensor(SensorPort.S3);
		//ir = new EV3IRSensor(SensorPort.S2);
		motorL.close();
		motorR.close();
		touchL.close();
		touchR.close();
	}
	
	
	
	public void rotationRight() {
		motorL.startSynchronization();
		motorL.rotate(167);
		motorR.rotate(-167);
		motorL.endSynchronization();
		Delay.msDelay(1000);
	}
	
	public void rotationLeft() {
		motorL.startSynchronization();
		motorR.rotate(167);
		motorL.rotate(-167);
		motorL.endSynchronization();
		Delay.msDelay(1000);
	}
	
	public void rotation180() {
		motorL.startSynchronization();
		motorR.rotate(332);
		motorL.rotate(-332);
		motorL.endSynchronization();
		Delay.msDelay(1100);
	}
	
	public void stop() {
		motorL.startSynchronization();
		motorL.stop();
		motorR.stop();
		motorL.endSynchronization();
	}
	
	public void ride(int l) {
		double angle = 360*l/(3.14159*5.6);
		motorL.startSynchronization();
		motorL.rotate((int)angle);
		motorR.rotate((int)angle);
		motorL.endSynchronization();
		Delay.msDelay(l*100);
	}
	
	
	
}
