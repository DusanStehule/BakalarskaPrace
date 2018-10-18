import java.util.LinkedList;

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

public class Ride4 {

	RegulatedMotor motorL;
	RegulatedMotor motorR;
	RegulatedMotor motorSmall;

	EV3UltrasonicSensor distanceSensor;
	EV3TouchSensor touch;
	EV3GyroSensor gyro;
	SensorMode touchM;
	SampleProvider distanceSampler;
	SampleProvider listenSampler;
	SampleProvider gyroSampler;

	double distance;

	float[] sampleDistance;
	float[] sampleTouch;
	float[] sampleGyro;

	Desk desk;

	public Ride4() {
		motorL = new EV3LargeRegulatedMotor(MotorPort.C);
		motorR = new EV3LargeRegulatedMotor(MotorPort.B);
		motorSmall = new EV3LargeRegulatedMotor(MotorPort.A);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		touch = new EV3TouchSensor(SensorPort.S2);
		distanceSensor = new EV3UltrasonicSensor(SensorPort.S3);
		gyro = new EV3GyroSensor(SensorPort.S4);
		distanceSampler = distanceSensor.getDistanceMode();
		sampleDistance = new float[distanceSampler.sampleSize()];
		touchM = touch.getTouchMode();
		sampleTouch = new float[touchM.sampleSize()];
		gyroSampler = gyro.getAngleAndRateMode();
		sampleGyro = new float[gyroSampler.sampleSize()];
		desk = new Desk(distanceSensor);
		// initialConditions();
		// start();
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
		motorSmall.resetTachoCount();
		motorSmall.rotate(-90); // nastavi US sensor doprava
		while (true) {
			rideWhile();
			rotate();
		}
	}

	/*
	 * Jede, dokud: -neni kam zatocit -nenarazi -jel by znovu, kde uz byl
	 */
	private void rideWhile() {
		int help;
		motorL.resetTachoCount();

		distanceSampler.fetchSample(sampleDistance, 0);
		touchM.fetchSample(sampleTouch, 0);
		help = desk.control(); // vraci 0, pokud tam robot jeste nebyl
		if (help == 0) {
			motorsForward();
		}

		do {
			help = desk.control(); // vraci 0, pokud tam robot jeste nebyl
			touchM.fetchSample(sampleTouch, 0);
		} while ((motorL.getTachoCount() < 190) && (help == 0) && (sampleTouch[0] == 0));

		if (sampleTouch[0] == 1) {
			motorL.startSynchronization();
			motorL.rotate(-30);
			motorR.rotate(-30);
			motorL.endSynchronization();
			Delay.msDelay(100);
			if ((motorL.getTachoCount() < 100) && (motorL.getTachoCount() > 0)) {
				desk.back();
			}
			desk.measureForward();
			help = 1;
			if (motorSmall.getTachoCount() < -80) {
				motorSmall.rotate(180);
			}
			measure();
		}

		if (help == 0) {
			Delay.msDelay(50);
			measure();
			desk.move();
		}

		help = desk.control(); // vraci 0, pokud tam robot jeste nebyl
		if (help == 1) {
			Delay.msDelay(250);
		}

		help = desk.control();
		desk.turnOffLight();
		motorL.resetTachoCount();

		do {
			if (motorL.getTachoCount() > 550) { // puvodne 550
				measure();
				desk.move();
				help = desk.control();
				desk.turnOffLight();
				motorL.resetTachoCount();
			}
			distanceSampler.fetchSample(sampleDistance, 0);
			touchM.fetchSample(sampleTouch, 0);
		} while ((sampleDistance[0] < 0.3) && (sampleTouch[0] == 0) && (help == 0));

		if (sampleTouch[0] == 1) {

			motorL.startSynchronization();
			motorL.rotate(-30);
			motorR.rotate(-30);
			motorL.endSynchronization();
			Delay.msDelay(100);
			if ((motorL.getTachoCount() < 100) && (motorL.getTachoCount() > 0)) {
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
			Delay.msDelay(270);
			desk.turnOffLight();
		}

		motorsStop();
		motorL.resetTachoCount();
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
		int angle = -90 - motorSmall.getTachoCount();
		int rot = 0;

		motorSmall.rotate(angle); // otoci US senzor doprava
		distanceSampler.fetchSample(sampleDistance, 0);
		helpRight = desk.controlPresenceRight(); // bude 1, pokud tam robot jeste nebyl (policko vpravo)
		helpRightEdge = desk.controlPresenceEdge(); // bude 1, pokud ma robot na prave strane bludiste
		helpForward = desk.control(); // bude 1, pokud tam robot uz byl (policko pred nim)
		helpEdge = desk.controlEdge(); // vraci 1, pokud je pred robotem okraj bludiste
		helpAround = desk.controlPresenceAround(); // bude 1, kdyz se robot nema kam hnout kvuli prekazce, nebo ze uz
													// tam byl
		// System.out.println("help " + helpRight + " " + helpForward + " " +
		// helpAround);
		if ((sampleDistance[0] > 0.3) && (helpRight == 1)) {
			rotationRight();
			System.out.println("rotation right A");
			rot = 1;
		}

		helpForward = desk.control(); // bude 1, pokud tam robot uz byl (policko pred nim)

		if (helpEdge == 1) {
			Delay.msDelay(1000);
		}

		if (((helpForward == 1) || (helpEdge == 1)) && (helpAround == 0) && (rot == 0)) {
			if (motorSmall.getTachoCount() < -80) {
				motorSmall.rotate(180); // otoci US senzor doleva
			}
			Delay.msDelay(200);
			measure();
			distanceSampler.fetchSample(sampleDistance, 0);
			if (sampleDistance[0] > 0.3) {
				rotationLeft();
				rot = 1;
				System.out.println("rotation left B");
			} else {
				rotation180();
				rot = 1;
				System.out.println("rotation 180 B");
			}
		}

		if ((helpRight == 0) && (helpAround == 0) && (helpForward == 1) && (rot == 0)) {
			if (motorSmall.getTachoCount() < -80) {
				motorSmall.rotate(180); // otoci US senzor doleva
			}
			Delay.msDelay(200);
			measure();
			distanceSampler.fetchSample(sampleDistance, 0);
			if (sampleDistance[0] > 0.3) {
				rotationLeft();
				rot = 1;
				System.out.println("rotation left C");
			} else if (helpRightEdge == 0) {
				rotation180();
				rot = 1;
				System.out.println("rotation 180 C");
			}
		}

		if (helpAround == 1) {
			System.out.println("help around");
			findField();
			System.out.println("konec cesty");
		}
		rot = 0;
	}

	private void findField() {
		int waySize = desk.findWay();
		int rot;
		motorsForward();
		rot = desk.rotationToWay();
		for (int i = 0; i < waySize; i++) {
			if (rot != 0) {
				motorsStop();
			}
			Delay.msDelay(100);
			switch (rot) {
			case -3:
				rotationLeft();
				motorsForward();
				break;
			case -2:
				rotation180();
				motorsForward();
				break;
			case -1:
				rotationRight();
				motorsForward();
				break;
			case 0:
				break;
			case 1:
				rotationLeft();
				motorsForward();
				break;
			case 2:
				rotation180();
				motorsForward();
				break;
			case 3:
				rotationRight();
				motorsForward();
				break;
			}
			motorL.resetTachoCount();
			if (i < waySize - 1) {
				rot = desk.rotationToWay();
			}
			
			if (((rot == -1) || (rot == 3)) && (i < waySize - 1)) {
				Delay.msDelay(400);
				do {
					distanceSampler.fetchSample(sampleDistance, 0);
					touchM.fetchSample(sampleTouch, 0);
				} while ((sampleDistance[0] < 0.3) && (sampleTouch[0] == 0));
				if (sampleDistance[0] > 0.4) {
					Delay.msDelay(470);
				}
			} else {
				do {
					touchM.fetchSample(sampleTouch, 0);
				} while ((motorL.getTachoCount() < 550) && (sampleTouch[0] == 0));
			}

			if (sampleTouch[0] == 1) {
				motorL.startSynchronization();
				motorL.rotate(-30);
				motorR.rotate(-30);
				motorL.endSynchronization();
				Delay.msDelay(100);
				desk.measureForward();
				if (motorSmall.getTachoCount() < -80) {
					motorSmall.rotate(180);
				}
				measure();
			}
			desk.move();
			if (motorL.getTachoCount() < 100) {
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
	 * udela jeden krok otocenim obou motoru o 573�
	 */
	public void oneStep() {
		oneStepRotate();
		desk.move();
	}

	/*
	 * rotuje oba motory o 573�
	 */
	private void oneStepRotate() {
		motorL.startSynchronization();
		motorL.rotate(573);
		motorR.rotate(573);
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

		while (sampleGyro[0] < 67) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}

		motorsStop();
		Delay.msDelay(300);
		gyroSampler.fetchSample(sampleGyro, 0);

		if (sampleGyro[0] < 80) {
			motorL.startSynchronization();
			motorL.backward();
			motorR.forward();
			motorL.endSynchronization();
			while (sampleGyro[0] < 80) {
				gyroSampler.fetchSample(sampleGyro, 0);
			}
			motorsStop();
			gyro.reset();
			Delay.msDelay(200);
		} else {
			gyro.reset();
		}

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

		while (sampleGyro[0] > -70) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}

		motorsStop();
		Delay.msDelay(300);
		gyroSampler.fetchSample(sampleGyro, 0);

		if (sampleGyro[0] > -80) {
			motorL.startSynchronization();
			motorL.forward();
			motorR.backward();
			motorL.endSynchronization();
			while (sampleGyro[0] > -80) {
				gyroSampler.fetchSample(sampleGyro, 0);
			}
			motorsStop();
			gyro.reset();
			Delay.msDelay(200);
		} else {
			gyro.reset();
		}

		desk.rotationR();

	}

	/*
	 * otoci se o 180�
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

		while (sampleGyro[0] < 159) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}

		motorsStop();
		gyro.reset();
		Delay.msDelay(200);

		desk.rotationB();

	}
}