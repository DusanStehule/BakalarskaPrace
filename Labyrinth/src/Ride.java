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

public class Ride {

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

	Desk desk;

	public Ride() {
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
		desk = new Desk(distanceSensor);
		// initialConditions();
		// start();
		motorSmall.setSpeed(700);
		motorSmall.resetTachoCount();
		motorSmall.rotate(-90); // nastavi US sensor doprava
		goToLabyrinth();
		motorL.close();
		motorR.close();
		distanceSensor.close();
		touch.close();
	}
	/*
	 * private void initialConditions() { desk.initialCondition(); }
	 */
	/*
	 * private void start() { motorSmall.resetTachoCount(); motorSmall.rotate(-90);
	 * // nastavi US sensor doprava rideWhile(); rotate(); if
	 * (motorSmall.getTachoCount() < -80) { motorSmall.rotate(180); } rideWhile();
	 * rotationLeft(); }
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
	 * Jede, dokud: -neni kam zatocit -nenarazi -jel by znovu, kde uz byl
	 */
	private void rideWhile() {
		int help;
		int reset = 0;
		int stop = 0;
		motorL.resetTachoCount();
		if (motorSmall.getTachoCount() > 80) {
			motorSmall.rotate(-180);
		}

		help = desk.control(); // vraci 0, pokud tam robot jeste nebyl
		if (help == 0) {
			motorsForward();
		}

		do {
			help = desk.control(); // vraci 0, pokud tam robot jeste nebyl
			touchM.fetchSample(sampleTouch, 0);
			resetM.fetchSample(sampleReset, 0);
		} while ((motorL.getTachoCount() < 250) && (help == 0) && (sampleTouch[0] == 0) && (sampleReset[0] == 0)); // puvodne 200

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
			if (motorSmall.getTachoCount() < -80) {
				motorSmall.rotate(180);
			}
			measure();
			stop = 1;

		}
		if (help == 0) {
			measure();
			desk.move();
		}
		
		if (sampleReset[0] == 1) {
			reset();
		}

		help = desk.control(); // vraci 0, pokud tam robot jeste nebyl
		if (help == 1) {
			Delay.msDelay(300);
		}
		help = desk.control();
		desk.turnOffLight();
		motorL.resetTachoCount();
		Delay.msDelay(200);
		do {
			if (motorL.getTachoCount() > 560) { // puvodne 540
				measure();
				desk.move();
				help = desk.control();
				desk.turnOffLight();
				motorL.resetTachoCount();
			}
			distanceSampler.fetchSample(sampleDistance, 0);
			touchM.fetchSample(sampleTouch, 0);
			resetM.fetchSample(sampleReset, 0);
		} while ((sampleDistance[0] < 0.3) && (sampleTouch[0] == 0) && (help == 0) && (stop == 0) && (sampleReset[0] == 0));
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
			if (motorSmall.getTachoCount() < -80) {
				motorSmall.rotate(180);
			}
			measure();
		}
		if (sampleDistance[0] > 0.3) {
			Delay.msDelay(430);
			desk.turnOffLight();
		}

		if (help == 1) {
			Delay.msDelay(200);
			desk.turnOffLight();
		}

		if (sampleReset[0] == 1) {
			reset();
		}
		
		motorsStop();
		motorL.resetTachoCount();
		Delay.msDelay(50);
	}

	/*
	 * zatoci doprava nebo doleva, kdyz: -to doprava nejde -vpravo uz byl
	 */
	private void rotate() {
		int helpRight;
		int helpRightEdge;
		int helpForward;
		int helpEdge;
		int helpAround;
		int rot = 0;

		if (motorSmall.getTachoCount() > 80) {
			motorSmall.rotate(-180); // otoci US senzor doprava
		}
		distanceSampler.fetchSample(sampleDistance, 0);
		helpRight = desk.controlPresenceRight(); // bude 1, pokud tam robot jeste nebyl (policko vpravo)
		helpRightEdge = desk.controlPresenceEdge(); // bude 1, pokud ma robot na prave strane bludiste
		helpForward = desk.control(); // bude 1, pokud tam robot uz byl (policko pred nim)
		helpEdge = desk.controlEdge(); // vraci 1, pokud je pred robotem okraj bludiste
		helpAround = desk.controlPresenceAround(); // bude 1, kdyz se robot nema kam hnout kvuli prekazce, nebo ze uz
													// tam byl
		// System.out.println("help " + helpRight + " " + helpForward + " " + helpEdge +
		// " " + helpAround);
		if ((sampleDistance[0] > 0.3) && (helpRight == 1)) {
			rotationRight();
			rot = 1;
		}
		helpForward = desk.control(); // bude 1, pokud tam robot uz byl (policko pred nim)

		if (((helpForward == 1) || (helpEdge == 1)) && (helpAround == 0) && (rot == 0)) {
			if (motorSmall.getTachoCount() < -80) {
				motorSmall.rotate(180); // otoci US senzor doleva
			}
			Delay.msDelay(100);
			measure();
			distanceSampler.fetchSample(sampleDistance, 0);
			if (sampleDistance[0] > 0.3) {
				rotationLeft();
				rot = 1;
			} else {
				rotation180();
				rot = 1;
			}
		}

		if ((helpRight == 0) && (helpAround == 0) && (helpForward == 1) && (rot == 0)) {
			if (motorSmall.getTachoCount() < -80) {
				motorSmall.rotate(180); // otoci US senzor doleva
			}
			Delay.msDelay(100);
			measure();
			distanceSampler.fetchSample(sampleDistance, 0);
			if (sampleDistance[0] > 0.3) {
				rotationLeft();
				rot = 1;
			} else if (helpRightEdge == 0) {
				rotation180();
				rot = 1;
			}
		}

		if (helpAround == 1) {
			findField();
		}
		rot = 0;
	}

	private void findField() {
		int waySize = desk.findWay();
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
				motorSmall.rotate(180);
				Delay.msDelay(100);
			}
			motorsForward();

			if ((rotNext == -1) || (rotNext == 3)) { // vpravo
				System.out.println("vpravo");
				Delay.msDelay(300);
				do {
					distanceSampler.fetchSample(sampleDistance, 0);
					touchM.fetchSample(sampleTouch, 0);
				} while ((sampleDistance[0] < 0.3) && (sampleTouch[0] == 0));
				if (sampleDistance[0] > 0.3) {
					Delay.msDelay(430);
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
					measure();
				}
			} else if ((rotNext == 1) || (rotNext == -3)) { // vlevo
				System.out.println("vlevo");
				Delay.msDelay(300);
				do {
					distanceSampler.fetchSample(sampleDistance, 0);
					touchM.fetchSample(sampleTouch, 0);
				} while ((sampleDistance[0] < 0.3) && (sampleTouch[0] == 0));
				if (sampleDistance[0] > 0.3) {
					Delay.msDelay(400);
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
					measure();
				}
			} else {
				do {
					touchM.fetchSample(sampleTouch, 0);
				} while ((motorL.getTachoCount() < 550) && (sampleTouch[0] == 0));
			}

			if (help == 0) {
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
	 * dle natoceni US senzoru meri bud nalevo nebo napravo
	 */
	private void measure() {
		if (motorSmall.getTachoCount() > 80) {
			desk.measureLeft();
		} else if (motorSmall.getTachoCount() < -80) {
			desk.measureRight();
		}
	}

	/*
	 * spusti oba motory dopredu
	 */
	private void motorsForward() {
		motorL.startSynchronization();
		motorL.setSpeed(600);
		motorR.setSpeed(600);
		motorL.forward();
		motorR.forward();
		motorL.endSynchronization();
	}

	/*
	 * zastavi oba motory
	 */
	private void motorsStop() {
		motorL.startSynchronization();
		motorL.stop();
		motorR.stop();
		motorL.endSynchronization();
	}

	/*
	 * udela jeden krok otocenim obou motoru o 573°
	 */
	public void oneStep() {
		oneStepRotate();
		desk.move();
	}

	/*
	 * rotuje oba motory o 573°
	 */
	private void oneStepRotate() {
		motorL.setSpeed(600);
		motorR.setSpeed(600);
		motorL.startSynchronization();
		motorL.rotate(580);
		motorR.rotate(580);
		motorL.endSynchronization();
		Delay.msDelay(1900);
	}

	/*
	 * zatoci doleva (gyro)
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

		while (sampleGyro[0] < 65) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}

		motorsStop();
		Delay.msDelay(100);
		gyroSampler.fetchSample(sampleGyro, 0);

		if (sampleGyro[0] < 82) {
			motorL.startSynchronization();
			motorL.backward();
			motorR.forward();
			motorL.endSynchronization();
			while (sampleGyro[0] < 80) {
				gyroSampler.fetchSample(sampleGyro, 0);
			}
			motorsStop();
		}

		gyro.reset();
		desk.rotationL();

	}

	/*
	 * zatoci doprava (gyro)
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

		while (sampleGyro[0] > -68) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}

		motorsStop();
		Delay.msDelay(100);
		gyroSampler.fetchSample(sampleGyro, 0);

		if (sampleGyro[0] > -82) {
			motorL.startSynchronization();
			motorL.forward();
			motorR.backward();
			motorL.endSynchronization();
			while (sampleGyro[0] > -80) {
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
		gyro.reset();
		desk.rotationB();
	}

	private void reset() {
		System.out.println("reset");
		motorsStop();
		desk.reset();
		Delay.msDelay(2000);
		goToLabyrinth();
	}
}