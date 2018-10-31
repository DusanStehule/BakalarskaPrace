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
		int motors = 0;
		int helpGhost;
		int stop = 0;
		motorL.resetTachoCount();
		if (motorSmall.getTachoCount() > 80) {
			motorSmall.rotate(-180);
		}
		help = desk.control(); // vraci 0, pokud tam robot jeste nebyl
		helpGhost = desk.controlGhost(); // vraci 1, pokud je na policku pred nim duch
		if (help == 0) {
			motorsForward();
			motors = 1;
		}

		do {
			help = desk.control(); // vraci 0, pokud tam robot jeste nebyl
			touchM.fetchSample(sampleTouch, 0);
			resetM.fetchSample(sampleReset, 0);
		} while ((motorL.getTachoCount() < 200) && (help == 0) && (sampleTouch[0] == 0) && (sampleReset[0] == 0)
				&& (motors == 1));

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
			stop = 1;

		}
		if (help == 0) {
			measure();
			desk.move();
		}

		if (sampleReset[0] == 1) {
			reset();
		}

		if (help == 1) {
			Delay.msDelay(300);
		}
		help = desk.control(); // vraci 0, pokud tam robot jeste nebyl
		helpGhost = desk.controlGhost(); // vraci 1, pokud je na policku pred nim duch

		motorL.resetTachoCount();
		do {
			if (!motorL.isMoving()) {
				motorsForward();
			}
			if (motorL.getTachoCount() > 510) {
				measure();
				desk.move();
				help = desk.control(); // vraci 0, pokud tam robot jeste nebyl
				helpGhost = desk.controlGhost(); // vraci 1, pokud je na policku pred nim duch
				motorL.resetTachoCount();
				distanceSampler.fetchSample(sampleDistance, 0);
			}

			if ((helpGhost == 1)) {
				helpGhost = beforeGhost();
			}

			touchM.fetchSample(sampleTouch, 0);
			resetM.fetchSample(sampleReset, 0);
			distanceSampler.fetchSample(sampleDistance, 0);
		} while ((sampleDistance[0] < 0.3) && (sampleTouch[0] == 0) && (help == 0) && (helpGhost == 0) && (stop == 0)
				&& (sampleReset[0] == 0));

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
			Delay.msDelay(300);
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

		helpRight = desk.controlPresenceRight(); // bude 1, pokud tam robot uz byl nebo je tam prekazka(policko vpravo)
		helpRightEdge = desk.controlPresenceEdge(); // bude 1, pokud ma robot na prave strane bludiste
		helpForward = desk.control(); // bude 1, pokud tam robot uz byl (policko pred nim)
		helpEdge = desk.controlEdge(); // vraci 1, pokud je pred robotem okraj bludiste
		helpAround = desk.controlPresenceAround(); // bude 1, kdyz se robot nema kam hnout kvuli prekazce, nebo ze uz
													// tam byl
		// System.out.println("help " + helpRight + " " + helpForward + " " + helpEdge +
		// " " + helpAround);
		Delay.msDelay(100);
		distanceSampler.fetchSample(sampleDistance, 0);
		Delay.msDelay(50);
		System.out.println("rotate " + helpRight + " " + sampleDistance[0]);
		if ((sampleDistance[0] > 0.3) && (helpRight == 0)) {
			markGhost(sampleDistance[0]);
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
				markGhost(sampleDistance[0]);
				rotationLeft();
			} else {
				rotation180();
			}
			rot = 1;
		}

		if ((helpRight == 1) && (helpAround == 0) && (helpForward == 1) && (rot == 0)) {
			if (motorSmall.getTachoCount() < -80) {
				motorSmall.rotate(180); // otoci US senzor doleva
			}
			Delay.msDelay(100);
			measure();
			distanceSampler.fetchSample(sampleDistance, 0);
			if (sampleDistance[0] > 0.3) {
				markGhost(sampleDistance[0]);
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

	/*
	 * BFS, hleda cestu a pak naviguje robota
	 */
	private void findField() {
		int waySize = desk.findWay();
		System.out.println("cesta " + waySize);
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
				Delay.msDelay(300);
				do {
					distanceSampler.fetchSample(sampleDistance, 0);
					touchM.fetchSample(sampleTouch, 0);
				} while ((sampleDistance[0] < 0.3) && (sampleTouch[0] == 0));
				if (sampleDistance[0] > 0.3) {
					Delay.msDelay(350);
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
	 * resi, co bude robot delat na policku pred duchem
	 */
	private int beforeGhost() {
		int helpGhost;
		int help = 0;
		motorL.resetTachoCount();
		motorsForward();
		while ((motorL.getTachoCount() < 300) && (sampleTouch[0] == 0) && (sampleDistance[0] < 0.3)) {
			touchM.fetchSample(sampleTouch, 0);
			distanceSampler.fetchSample(sampleDistance, 0);
		}
		if (sampleDistance[0] > 0.3) {
			help = 1;
		} 
	//	measure();
	//	System.out.println("measure " + desk.getRow() + " " + desk.getColumn());
		motorsStop();
		if (motorSmall.getTachoCount() < -80) {
			motorSmall.rotate(90);
		} else if (motorSmall.getTachoCount() > 80) {
			motorSmall.rotate(-90);
		}
		Delay.msDelay(100);
		distanceSampler.fetchSample(sampleDistance, 0);
		markGhost(sampleDistance[0]);
		helpGhost = desk.controlGhost(); // vraci 1, pokud je na policku pred nim duch
		System.out.println(helpGhost + " duch pred robotem");
		if (helpGhost == 1) {
			motorL.resetTachoCount();
			motorsForward();
			while (motorL.getTachoCount() < (sampleDistance[0] - 0.15) * 2100) {
			}
			motorsStop();
			desk.measureForward();
		} else {
			if (motorSmall.getTachoCount() > -70) {
				motorSmall.rotate(-90); // otoci US sensor doprava
				Delay.msDelay(200);
			}
			motorsForward();
			while ((motorL.getTachoCount() < 510) && (sampleTouch[0] == 0) && (sampleDistance[0] < 0.3)) {
				touchM.fetchSample(sampleTouch, 0);
				distanceSampler.fetchSample(sampleDistance, 0);
			}
			if (help == 0) {
				measure();
				desk.move();
			}
		}
		motorL.resetTachoCount();
		if (motorSmall.getTachoCount() > -70) {
			motorSmall.rotate(-90); // otoci US sensor doprava
			Delay.msDelay(200);
		}
		return helpGhost;
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
	 * vezme vzdalenost a smer a z ni spocita, kde by mohl byt duch
	 */

	private void markGhost(double length) {
		int number = 0;
		int directH = 0;
		if (motorSmall.getTachoCount() > 80) { // kouka doleva
			directH = -1;
		} else if ((motorSmall.getTachoCount() < 20) && (motorSmall.getTachoCount() > -20)) { // kouka rovne
			directH = 0;
		} else if (motorSmall.getTachoCount() < -80) { // kouka doprava
			directH = 1;
		}
		number = (int) ((length - 0.1) / 0.28) + 1;
		// System.out.println("number " + number + " " + length);
		desk.markGhost(number, directH);
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
		motorL.startSynchronization();
		motorL.setSpeed(600);
		motorR.setSpeed(600);
		motorL.rotate(600);
		motorR.rotate(600);
		motorL.endSynchronization();
		Delay.msDelay(1300);
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

		while (sampleGyro[0] < 63) {
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
			while (sampleGyro[0] < 82) {
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
			while (sampleGyro[0] > -82) {
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

	private void reset() {
		System.out.println("reset");
		motorsStop();
		desk.reset();
		Delay.msDelay(2000);
		goToLabyrinth();
	}
}