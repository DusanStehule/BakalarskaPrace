import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Ride {

	RegulatedMotor motorL;
	RegulatedMotor motorR;
	EV3ColorSensor color;
	EV3TouchSensor touch;
	SampleProvider colorSampler;
	SensorMode touchM;

	float[] sampleTouch;
	float[] sampleColor;

	double P = 505; // 505, 530
	double D = 5500; // 5500, 5900
	// bílá: 0.48
	// èerná: 0.03
	double target = 0.24;
	double last = 0;
	int speed = 290; //290, 300

	public Ride() {
		motorL = new EV3LargeRegulatedMotor(MotorPort.B);
		motorR = new EV3LargeRegulatedMotor(MotorPort.C);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		touch = new EV3TouchSensor(SensorPort.S1);
		color = new EV3ColorSensor(SensorPort.S2);
		touchM = touch.getTouchMode();
		colorSampler = color.getRedMode();
		sampleTouch = new float[touchM.sampleSize()];
		sampleColor = new float[colorSampler.sampleSize()];
		ride();
		color.close();
		touch.close();
	}

	private void ride() {
		/*
		 * touchM.fetchSample(sampleTouch, 0); while (sampleTouch[0] == 0) {
		 * touchM.fetchSample(sampleTouch, 0); }
		 */
		double error = 0;
		double derivation = 0;

		while (true) {
			colorSampler.fetchSample(sampleColor, 0);
			error = sampleColor[0] - target;
			derivation = sampleColor[0] - last;
			if ((error > 0.1) || (error < -0.1)) {
				motorL.setSpeed((int) (speed + P * error - D * derivation));
				motorR.setSpeed((int) (speed - P * error + D * derivation));
			} else {
				motorL.setSpeed((int) (speed + 250 * error)); //200
				motorR.setSpeed((int) (speed - 250 * error)); //200
			}
			
			motorL.startSynchronization();
			motorL.forward();
			motorR.forward();
			motorL.endSynchronization();
			last = sampleColor[0];
		}
	}

}
