import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Ride {

	RegulatedMotor motorL;
	RegulatedMotor motorR;
	EV3ColorSensor color;
	SampleProvider colorSampler;
	float[] sampleColor;
	
	double P = 500;
	double I = 0;
	double D = 1000;
	//bílá: 0.45
	//èerná: 0.03
	double target = 0.24;
	double last = 0;
	int speed = 500;

	public Ride() {
		motorL = new EV3LargeRegulatedMotor(MotorPort.B);
		motorR = new EV3LargeRegulatedMotor(MotorPort.C);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		color = new EV3ColorSensor(SensorPort.S2);
		colorSampler = color.getRedMode();
		sampleColor = new float[colorSampler.sampleSize()];
		motorL.setSpeed(speed);
		motorR.setSpeed(speed);
		motorL.forward();
		motorR.forward();
		Delay.msDelay(100000);
	//	ride();
		color.close();
	}

	private void ride() {
		
		double error = 0;
		double integral = 0;
		double derivation = 0;
		motorL.startSynchronization();
		motorL.setSpeed(speed);
		motorR.setSpeed(speed);
		motorL.forward();
		motorR.forward();
		motorL.endSynchronization();
		colorSampler.fetchSample(sampleColor, 0);
		
		while (true) { //vyjede doprava na bílou
			colorSampler.fetchSample(sampleColor, 0);
			System.out.println(sampleColor[0]);
			error = sampleColor[0] - target;  //0.2
			integral += error;
			derivation = sampleColor[0] - last; //0.45 - 0.4 = 0.05 
			motorL.startSynchronization();
			motorL.setSpeed((int) (speed - P * error - I * integral + D * derivation));
			motorR.setSpeed((int) (speed + P * error + I * integral - D * derivation));
			motorL.forward();
			motorR.forward();
			motorL.endSynchronization();
			last = sampleColor[0];
		}
		
	}

}
