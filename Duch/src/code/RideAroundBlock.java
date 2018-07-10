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
		touchLeft = touchL.getTouchMode();
			touchRight = touchR.getTouchMode();
			rangeSampler = irSensor.getDistanceMode();
			float[] sampleL = new float[touchLeft.sampleSize()];
			float[] sampleR = new float[touchRight.sampleSize()];
			float[] lastRange = new float[rangeSampler.sampleSize()];
			
			while (true) {
			System.out.println("ahoj");
			motorL.startSynchronization();
			motorL.stop();
			motorR.stop();
			motorL.forward();
			motorR.forward();
			motorL.endSynchronization();

			do{
				touchLeft.fetchSample(sampleL, 0);
				touchRight.fetchSample(sampleR, 0);
				rangeSampler.fetchSample(lastRange, 0);
				
			} while (((sampleL[0] == 0) || (sampleR[0] == 0)) && (lastRange[0] < 0.2) );
			
			if ((sampleL[0] != 0) && (sampleR[0] != 0)) {
				robot.stop();
				motorL.startSynchronization();
				motorL.rotateTo(-180);
				motorR.rotateTo(-180);
				motorL.endSynchronization();
				Delay.msDelay(600);
				robot.rotationRight();
				System.out.println("tocim doprava");
			}
			
			if (lastRange[0] > 0.2) {
				robot.ride(14);
				robot.rotationLeft();
				robot.ride(15);
				System.out.println("tocim doleva");
				motorL.startSynchronization();
				motorL.forward();
				motorR.forward();
				motorL.endSynchronization();
			}
		}
	}

}
