import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class Ride {

	RegulatedMotor motorL;
	RegulatedMotor motorR;
	EV3ColorSensor color;
	SampleProvider colorSampler;
	float[] sampleColor;
	
	double P = 600;
	double I = 24;
	double D = 1000;
	double target = 0.33;
	double last = 0;
	int speed = 300;

	public Ride() {
		motorL = new EV3LargeRegulatedMotor(MotorPort.B);
		motorR = new EV3LargeRegulatedMotor(MotorPort.C);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		color = new EV3ColorSensor(SensorPort.S2);
		colorSampler = color.getRedMode();
		sampleColor = new float[colorSampler.sampleSize()];
		ride();
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
		
		while (true) {
			colorSampler.fetchSample(sampleColor, 0);
			error = sampleColor[0] - target;
			integral += error;
			derivation = sampleColor[0] - last; 
			motorL.startSynchronization();
			motorL.setSpeed((int) (speed - P * error - I * integral - D * derivation));
			motorR.setSpeed((int) (speed + P * error + I * integral + D * derivation));
			motorL.endSynchronization();
			last = sampleColor[0];
		}
		
	}

}
