import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
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

	// docela dobré: 2200, 2350, 2450, 2625, 2800 pro rychlost 450 a P=520
	// p=430, d=3000, s=340, t=0.3

	double P = 470; //470
	double D = 4875; //od 2000 výše; 2525 a 2800 a 3050 a 3400 a 3650 a 4350 a 4625 a 4800 dobré
	// bílá: 0.48
	// èerná: 0.03
	double target = 0.25;
	double last = 0;
	int speed = 380;

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
			motorL.startSynchronization();
			if ((error > 0.1) || (error < -0.1)) {
				motorL.setSpeed((int) (speed + P * error - D * derivation));
				motorR.setSpeed((int) (speed - P * error + D * derivation));
			} else {
				motorL.setSpeed((int) (speed + 200 * error));
				motorR.setSpeed((int) (speed - 200 * error));
			}

			motorL.forward();
			motorR.forward();
			motorL.endSynchronization();
			last = sampleColor[0];
		}
	}

}
