import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Ride1 {

	RegulatedMotor motorL;
	RegulatedMotor motorR;
	RegulatedMotor motorSmall;

	EV3UltrasonicSensor distanceSensor;
	EV3TouchSensor touch;
	EV3TouchSensor reset;
	EV3GyroSensor gyro;
	SensorMode touchM;
	SensorMode resetM;
	SampleProvider distanceSampler;
	SampleProvider listenSampler;
	SampleProvider gyroSampler;

	double distance;

	float[] sampleDistance;
	float[] sampleTouch;
	float[] sampleReset;
	float[] sampleGyro;

	Desk1 desk;

	public Ride1() {
		motorL = new EV3LargeRegulatedMotor(MotorPort.C);
		motorR = new EV3LargeRegulatedMotor(MotorPort.B);
		motorSmall = new EV3LargeRegulatedMotor(MotorPort.A);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		reset = new EV3TouchSensor(SensorPort.S1);
		touch = new EV3TouchSensor(SensorPort.S2);
		distanceSensor = new EV3UltrasonicSensor(SensorPort.S3);
		gyro = new EV3GyroSensor(SensorPort.S4);
		distanceSampler = distanceSensor.getDistanceMode();
		sampleDistance = new float[distanceSampler.sampleSize()];
		touchM = touch.getTouchMode();
		resetM = reset.getTouchMode();
		sampleTouch = new float[touchM.sampleSize()];
		sampleReset = new float[resetM.sampleSize()];
		gyroSampler = gyro.getAngleAndRateMode();
		sampleGyro = new float[gyroSampler.sampleSize()];
		desk = new Desk1(distanceSensor);
		motorSmall.setSpeed(700);
		motorSmall.resetTachoCount();
		motorSmall.rotate(-90); // rotate US sensor to the right
		goToLabyrinth();
		motorL.close();
		motorR.close();
		distanceSensor.close();
		touch.close();
		gyro.close();
	}

	/*
	 * endless cycle
	 */
	private void goToLabyrinth() {
		resetM.fetchSample(sampleReset, 0);
		while (sampleReset[0] == 0) {
			resetM.fetchSample(sampleReset, 0);
		}
		Delay.msDelay(1000);

		oneStep();
		while (true) {
			rotate();
			rideWhile();
		}
	}

	/*
	 * robot rides until:
	 * -> he can't turn right
	 * -> he will not bump into wall or barrier
	 * -> he will not drive to field, where he was already
	 */
	private void rideWhile() {
		int help;
		int motors = 0;
		int helpRightEdge;
		int helpLeftPresence = 1;
		int stop = 0;
		motorL.resetTachoCount();
		helpRightEdge = desk.controlRightEdge(); 
		
		if ((motorSmall.getTachoCount() > 80) && (helpRightEdge == 0)) {
			motorSmall.rotate(-180); // rotate US sensor to the right
		}
		if ((motorSmall.getTachoCount() < -80) && (helpRightEdge == 1)) {
			motorSmall.rotate(180); // rotate US sensor to the left
		}
		help = desk.controlForward(); // 1 if robot was on the field forward 

		if (help == 0) {
			motorsForward();
			motors = 1;
		}

		do {
			if ((motorL.getTachoCount() > 160)) {
				measure();
			}
			touchM.fetchSample(sampleTouch, 0);
			resetM.fetchSample(sampleReset, 0);

		} while ((motorL.getTachoCount() < 190) && (help == 0) && (sampleTouch[0] == 0) && (sampleReset[0] == 0)
				&& (motors == 1));

		if (sampleTouch[0] == 1) {
			motorL.startSynchronization();
			motorL.rotate(-30);
			motorR.rotate(-30);
			motorL.endSynchronization();
			Delay.msDelay(50);
			motorsStop();
			desk.measureForward();
			help = 1;
			if (motorSmall.getTachoCount() < -80) {
				motorSmall.rotate(180);
			}
			stop = 1;

		}
		if (help == 0) {
			desk.move();
			helpLeftPresence = desk.controlPresenceLeftPresence(); // 1 if robot was on the field left
		}

		if (sampleReset[0] == 1) {
			reset();
		}

		help = desk.controlForward(); // 1 if robot was on the field forward

		motorL.resetTachoCount();
		Delay.msDelay(200); 
		distanceSampler.fetchSample(sampleDistance, 0);
		touchM.fetchSample(sampleTouch, 0);
		resetM.fetchSample(sampleReset, 0);

		while (((sampleDistance[0] < 0.3) || ((helpRightEdge == 1) && (helpLeftPresence == 1))) && (sampleTouch[0] == 0)
				&& (help == 0) && (stop == 0) && (sampleReset[0] == 0)) {
			if ((!motorL.isMoving()) && (stop == 0)) {
				motorsForward();
			}
			if (motorL.getTachoCount() > 540) { 
				desk.move();
				help = desk.controlForward(); // 1 if robot was on the field forward
				helpLeftPresence = desk.controlPresenceLeftPresence(); // 1 if robot was on the field left
				motorL.resetTachoCount();
			}

			if ((motorL.getTachoCount() > 450) && (motorL.getTachoCount() < 490)) {
				measure();
			}

			touchM.fetchSample(sampleTouch, 0);
			resetM.fetchSample(sampleReset, 0);
			distanceSampler.fetchSample(sampleDistance, 0);
		}

		if (sampleDistance[0] > 0.3) {
			if (motorL.getTachoCount() > 450) {
				desk.move();
			}
			Delay.msDelay(370);
			desk.turnOffLight();
		}

		if (sampleTouch[0] == 1) {
			motorL.startSynchronization();
			motorL.rotate(-30);
			motorR.rotate(-30);
			motorL.endSynchronization();
			Delay.msDelay(50);
			if ((motorL.getTachoCount() < 170) && (motorL.getTachoCount() > 0)) {
				desk.back();
			}
			desk.measureForward();
			help = 1;
			if (motorSmall.getTachoCount() < -80) { // turn US sensor to the left
				motorSmall.rotate(180);
			}
		}

		if (help == 1) {
			Delay.msDelay(210);
		}

		if (sampleReset[0] == 1) {
			reset();
		}

		motorsStop();
		motorL.resetTachoCount();
		Delay.msDelay(50);
	}

	/*
	 * this procedure determines which direction to turn
	 */
	private void rotate() {
		int helpRight;
		int helpRightEdge;
		int helpRightPresence;
		int helpLeftPresence;
		int helpForward;
		int helpEdge;
		int helpAround;
		int rot = 0;
		helpRightEdge = desk.controlRightEdge(); 

		if ((motorSmall.getTachoCount() > 80) && (helpRightEdge == 0)) {
			motorSmall.rotate(-180); // turn US sensor to the right
			Delay.msDelay(50);
		} else if ((motorSmall.getTachoCount() > -80) && (helpRightEdge == 0)) {
			motorSmall.rotate(-90); // turn US sensor right
			Delay.msDelay(50);
		}
		distanceSampler.fetchSample(sampleDistance, 0);

		helpRight = desk.controlPresenceRight(); // 1 if robot was there or its barrier there
		helpRightEdge = desk.controlRightEdge(); 
		helpLeftPresence = desk.controlPresenceLeftPresence(); // 1 if robot was on the field left
		helpRightPresence = desk.controlPresenceRightPresence(); // bude 1, pokud tam robot uz byl
		helpForward = desk.controlForward(); // 1 if robot was on the field right
		helpEdge = desk.controlEdge(); // vraci 1, pokud je pred robotem okraj bludiste
		helpAround = desk.controlPresenceAround(); // 1 if robot was on the field forward, back, left and right or is barrier forward, back, left and right

		measure();
		if ((sampleDistance[0] > 0.3) && (helpRightPresence == 0) && (helpRightEdge == 0)) {
			rotationRight();
			rot = 1;
		}
		if ((sampleDistance[0] > 0.3) && (helpRightPresence == 0) && (helpLeftPresence == 0) && (helpRightEdge == 1)) {
			rotationLeft();
			rot = 1;
		}

		helpForward = desk.controlForward(); // 1 if robot was on the field forward 
		if (((helpForward == 1) || (helpEdge == 1)) && (helpAround == 0) && (rot == 0)) {
			System.out.println("xxxAxxx " + helpForward + " " + helpEdge);
			if (motorSmall.getTachoCount() < -80) {
				motorSmall.rotate(180); // turn US sensor to the left
			}
			Delay.msDelay(50);
			measure();
			distanceSampler.fetchSample(sampleDistance, 0);
			if (sampleDistance[0] > 0.3) {
				rotationLeft();
				measure();
			} else {
				rotation180();
				measure();
			}
			rot = 1;
		}

		if ((helpRight == 1) && (helpAround == 0) && (helpForward == 1) && (rot == 0)) {
			System.out.println("xxxBxxx");
			if (motorSmall.getTachoCount() < -80) {
				motorSmall.rotate(180); // turn US sensor to the left
			}
			Delay.msDelay(50);
			measure();
			distanceSampler.fetchSample(sampleDistance, 0);
			if (sampleDistance[0] > 0.3) {
				rotationLeft();
				measure();
			} else if (helpRightEdge == 0) {
				rotation180();
				measure();
			}
			rot = 1;
		}

		if (helpAround == 1) {
			findField();
		}
		rot = 0;
	}

	/*
	 * this procedure find way and than navigate robot
	 */
	private void findField() {
		int waySize = desk.findWay();
		System.out.println("way " + waySize);
		int help = 0;
		int rot;
		int rotNext;
		motorsForward();
		for (int i = 0; i < waySize; i++) {
			rot = desk.rotationToWay();
			if (rot != 0) {
				motorsStop();
			}
			switch (rot) {
			case -3:
				rotationLeft();
				break;
			case -2:
				rotation180();
				break;
			case -1:
				rotationRight();
				break;
			case 0:
				break;
			case 1:
				rotationLeft();
				break;
			case 2:
				rotation180();
				break;
			case 3:
				rotationRight();
				break;
			}
			motorL.resetTachoCount();
			rotNext = desk.getRotation();
			if (((rotNext == 1) || (rotNext == -3)) && (motorSmall.getTachoCount() < -80)) {
				motorSmall.rotate(180); // turn US sensor to the left
				Delay.msDelay(50);
			}
			motorsForward();

			if ((rotNext == -1) || (rotNext == 3)) { // right
				Delay.msDelay(250);
				distanceSampler.fetchSample(sampleDistance, 0);
				while ((sampleDistance[0] < 0.3) && (sampleTouch[0] == 0) && (motorL.getTachoCount() < 570)) {
					distanceSampler.fetchSample(sampleDistance, 0);
					touchM.fetchSample(sampleTouch, 0);
				}
				if (sampleDistance[0] > 0.3) {
					Delay.msDelay(300);
				}
				if (sampleTouch[0] == 1) {
					motorL.startSynchronization();
					motorL.rotate(-30);
					motorR.rotate(-30);
					motorL.endSynchronization();
					Delay.msDelay(50);
					desk.measureForward();
					help = 1;
					if (motorSmall.getTachoCount() < -80) {
						motorSmall.rotate(180);
					}
					Delay.msDelay(50);
				}
			} else if ((rotNext == 1) || (rotNext == -3)) { // left
				Delay.msDelay(250);
				distanceSampler.fetchSample(sampleDistance, 0);
				while ((sampleDistance[0] < 0.3) && (sampleTouch[0] == 0) && (motorL.getTachoCount() < 570)) {
					distanceSampler.fetchSample(sampleDistance, 0);
					touchM.fetchSample(sampleTouch, 0);
				}
				if (sampleDistance[0] > 0.3) {
					Delay.msDelay(300);
				}
				if (sampleTouch[0] == 1) {
					motorL.startSynchronization();
					motorL.rotate(-30);
					motorR.rotate(-30);
					motorL.endSynchronization();
					Delay.msDelay(50);
					desk.measureForward();
					help = 1;
					if (motorSmall.getTachoCount() > 80) {
						motorSmall.rotate(-180);
					}
				}
			} else {
				while ((motorL.getTachoCount() < 540) && (sampleTouch[0] == 0)) {
					if ((motorL.getTachoCount() > 450) && (motorL.getTachoCount() < 490)) {
						measure();
					}
					touchM.fetchSample(sampleTouch, 0);
				}
				if (sampleTouch[0] == 1) {
					motorL.startSynchronization();
					motorL.rotate(-30);
					motorR.rotate(-30);
					motorL.endSynchronization();
					Delay.msDelay(50);
					if ((motorL.getTachoCount() < 170) && (motorL.getTachoCount() > 0)) {
						 desk.back();
					}
					desk.measureForward();
				}
			}

			if (help == 0) {
				measure();
				desk.move();
				desk.turnOffLight();
			}

			if ((motorL.getTachoCount() < 200) && (motorL.getTachoCount() > 0)) {
				desk.back();
			}
		}
		desk.eraseWay();
	}

	/*
	 * according to the rotation of ultrasonic sensor measure this procedure left or right
	 */
	private void measure() {
		if (motorSmall.getTachoCount() > 80) {
			desk.measureLeft();
		} else if (motorSmall.getTachoCount() < -80) {
			desk.measureRight();
		}
	}

	/*
	 * turn on both motors to the forward
	 */
	private void motorsForward() {
		motorL.startSynchronization();
		motorL.setSpeed(700);
		motorR.setSpeed(700);
		motorL.forward();
		motorR.forward();
		motorL.endSynchronization();
	}

	/*
	 * stop both motors
	 */
	private void motorsStop() {
		motorL.startSynchronization();
		motorL.stop();
		motorR.stop();
		motorL.endSynchronization();
	}

	/*
	 * do one step
	 */
	public void oneStep() {
		oneStepRotate();
		desk.move();
	}

	/*
	 * rotate both motors about 600 degrees 
	 */
	private void oneStepRotate() {
		motorL.startSynchronization();
		motorL.setSpeed(700);
		motorR.setSpeed(700);
		motorL.rotate(600);
		motorR.rotate(600);
		motorL.endSynchronization();
		Delay.msDelay(1100);
	}

	/*
	 * turn left with gyroscope 
	 */
	private void rotationLeft() {
		gyro.reset();
		motorL.setSpeed(400);
		motorR.setSpeed(400);
		gyroSampler.fetchSample(sampleGyro, 0);
		motorL.startSynchronization();
		motorL.backward();
		motorR.forward();
		motorL.endSynchronization();

		while (sampleGyro[0] < 69) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}

		motorsStop();
		Delay.msDelay(120);
		gyroSampler.fetchSample(sampleGyro, 0);

		if (sampleGyro[0] < 85) {
			motorL.startSynchronization();
			motorL.backward();
			motorR.forward();
			motorL.endSynchronization();
			Delay.msDelay(30);
			while (sampleGyro[0] < 75) {
				gyroSampler.fetchSample(sampleGyro, 0);
			}
			motorsStop();
		}

		gyro.reset();
		desk.rotationL();

	}

	/*
	 * turn right with gyroscope 
	 */
	private void rotationRight() {
		motorL.setSpeed(400);
		motorR.setSpeed(400);
		gyro.reset();
		gyroSampler.fetchSample(sampleGyro, 0);
		motorL.startSynchronization();
		motorL.forward();
		motorR.backward();
		motorL.endSynchronization();

		while (sampleGyro[0] > -69) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}

		motorsStop();
		Delay.msDelay(120);
		gyroSampler.fetchSample(sampleGyro, 0);

		if (sampleGyro[0] > -85) {
			motorL.startSynchronization();
			motorL.forward();
			motorR.backward();
			motorL.endSynchronization();
			Delay.msDelay(30);
			while (sampleGyro[0] > -75) {
				gyroSampler.fetchSample(sampleGyro, 0);
			}
			motorsStop();
		}

		gyro.reset();
		desk.rotationR();
	}

	/*
	 * otoci se o 180°
	 */
	private void rotation180() {
		motorL.setSpeed(400);
		motorR.setSpeed(400);
		gyro.reset();
		gyroSampler.fetchSample(sampleGyro, 0);
		motorL.startSynchronization();
		motorL.backward();
		motorR.forward();
		motorL.endSynchronization();

		while (sampleGyro[0] < 150) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}

		motorsStop();
		Delay.msDelay(120);
		gyroSampler.fetchSample(sampleGyro, 0);

		if (sampleGyro[0] < 172) {
			motorL.startSynchronization();
			motorL.backward();
			motorR.forward();
			motorL.endSynchronization();
			Delay.msDelay(30);
			while (sampleGyro[0] < 170) {
				gyroSampler.fetchSample(sampleGyro, 0);
			}
			motorsStop();
		}

		gyro.reset();
		desk.rotationB();
	}

	/*
	 * reset robot's coordinates
	 */
	private void reset() {
		System.out.println("reset");
		motorsStop();
		desk.reset();
		Delay.msDelay(2000);
		goToLabyrinth();
	}
}