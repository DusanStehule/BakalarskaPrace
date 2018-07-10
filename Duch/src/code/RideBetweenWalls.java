package code;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class RideBetweenWalls {

	Robot robot;
	RegulatedMotor motorL;
	RegulatedMotor motorR;
	EV3TouchSensor touchL;
	EV3TouchSensor touchR;
	SensorMode touchLeft;
	SensorMode touchRight;
	boolean done;
	
	public RideBetweenWalls() {
		robot = new Robot();
		motorL = new EV3LargeRegulatedMotor(MotorPort.A);
		motorR = new EV3LargeRegulatedMotor(MotorPort.B);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		touchL = new EV3TouchSensor(SensorPort.S1);
		touchR = new EV3TouchSensor(SensorPort.S3);
		done = false;
		ride();
		motorL.close();
		motorR.close();
		touchL.close();
		touchR.close();
	}
	
	public void ride() {
		while (!done) {
			motorL.startSynchronization();
			motorL.forward();
			motorR.forward();
			motorL.endSynchronization();
			
			touchLeft = touchL.getTouchMode();
			touchRight = touchR.getTouchMode();
			float[] sampleL = new float[touchLeft.sampleSize()];
			float[] sampleR = new float[touchRight.sampleSize()];
			
			do{
				touchLeft.fetchSample(sampleL, 0);
				touchRight.fetchSample(sampleR, 0);
				if (Button.readButtons() != 0) {
		            done = true;
		        }
				
			} while ((sampleL[0] == 0) || (sampleR[0] == 0));
			
			System.out.println("otacim se");
			motorL.startSynchronization();
			motorL.stop();
			motorR.stop();
			motorL.rotateTo(-163);
			motorR.rotateTo(-163);
			motorL.endSynchronization();
			Delay.msDelay(600);
			
			robot.rotation180();

			if (Button.readButtons() != 0) {
	            done = true;
	        }

		} 
		System.out.println("CLOSING PORTS");
		
		touchL.close();
		touchR.close();
	}
	
	
	
}
