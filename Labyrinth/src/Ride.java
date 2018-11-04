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
		help = desk.controlForward(); // vraci 0, pokud tam robot jeste nebyl
		helpGhost = desk.controlGhost(); // vraci 1, pokud je na policku pred nim duch
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
			
		} while ((motorL.getTachoCount() < 200) && (help == 0) && (sampleTouch[0] == 0) && (sampleReset[0] == 0)
				&& (motors == 1));

		if (motorL.getTachoCount() > 200) {
			System.out.println("ride1 TachoCount");
		}
		if (help == 1) {
			System.out.println("ride1 help");
		}
		if (sampleTouch[0] == 1) {
			System.out.println("ride1 sTouch");
		}
		if (sampleReset[0] == 1) {
			System.out.println("ride1 sReset");
		}
		if (motors == 0) {
			System.out.println("ride1 motors");
		}
		
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
		}

		if (sampleReset[0] == 1) {
			reset();
		}
		
		help = desk.controlForward(); // vraci 0, pokud tam robot jeste nebyl
	//	helpGhost = desk.controlGhost(); // vraci 1, pokud je na policku pred nim duch
		
		if (help == 1) {
			Delay.msDelay(50);
		}
		
		motorL.resetTachoCount();
		Delay.msDelay(230);
		distanceSampler.fetchSample(sampleDistance, 0);
		touchM.fetchSample(sampleTouch, 0);
		resetM.fetchSample(sampleReset, 0);
		helpGhost = desk.controlGhost(); // vraci 1, pokud je na policku pred nim duch
		
		while ((sampleDistance[0] < 0.3) && (sampleTouch[0] == 0) && (help == 0) && (helpGhost == 0) && (stop == 0)
				&& (sampleReset[0] == 0)) {
			if ((!motorL.isMoving()) && (stop == 0)) {
				motorsForward();
			}
			if (motorL.getTachoCount() > 540) {
				desk.move();
				help = desk.controlForward(); // vraci 0, pokud tam robot jeste nebyl
				helpGhost = desk.controlGhost(); // vraci 1, pokud je na policku pred nim duch
				motorL.resetTachoCount();
			}
			
			if ((helpGhost == 1)) {
				System.out.println("beforeGhost");
				helpGhost = beforeGhost();
			}
			
			if ((motorL.getTachoCount() > 450) && (motorL.getTachoCount() < 490)) {
				measure();
			}

			touchM.fetchSample(sampleTouch, 0);
			resetM.fetchSample(sampleReset, 0);
			distanceSampler.fetchSample(sampleDistance, 0);
		} 
		
		if (help == 1) {
			System.out.println("ride2 help");
		}
		
		if (stop == 1) {
			System.out.println("ride1 stop");
		}
		
		if (sampleDistance[0] > 0.3) {
			System.out.println("ride2 sDistance");
		}
		
		if (helpGhost == 1) {
			System.out.println("ride2 helpGhost");
		}
		
		if (sampleTouch[0] == 1) {
			System.out.println("ride2 sTouch");
		}
		if (sampleReset[0] == 1) {
			System.out.println("ride2 sReset");
		}

		if (sampleDistance[0] > 0.3) {
			Delay.msDelay(430);
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
			if (motorSmall.getTachoCount() < -80) { //otoci US sensor doleva
				motorSmall.rotate(180);
			}
		}

		if (help == 1) {
			Delay.msDelay(300);
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
		int helpRightPresence;
		int helpForward;
		int helpEdge;
		int helpAround;
		int helpGhost;
		int rot = 0;

		if (motorSmall.getTachoCount() > 80) {
			motorSmall.rotate(-180); // otoci US senzor doprava
			Delay.msDelay(100);
			distanceSampler.fetchSample(sampleDistance, 0);
		//	Delay.msDelay(100);
		} else if (motorSmall.getTachoCount() > -80) {
			motorSmall.rotate(-90); // otoci US senzor doprava
			Delay.msDelay(100);
			distanceSampler.fetchSample(sampleDistance, 0);
		//	Delay.msDelay(100);
		} else {
		//	Delay.msDelay(100);
			distanceSampler.fetchSample(sampleDistance, 0);
		}

		helpRight = desk.controlPresenceRight(); // bude 1, pokud tam robot uz byl nebo je tam prekazka(policko vpravo)
		helpRightEdge = desk.controlPresenceEdge(); // bude 1, pokud ma robot na prave strane bludiste
		helpRightPresence = desk.controlPresenceRightPresence(); // bude 1, pokud tam robot uz byl
		helpForward = desk.controlForward(); // bude 1, pokud tam robot uz byl (policko pred nim)
		helpEdge = desk.controlEdge(); // vraci 1, pokud je pred robotem okraj bludiste
		helpAround = desk.controlPresenceAround(); // bude 1, kdyz se robot nema kam hnout kvuli prekazce, nebo ze uz
													// tam byl
		helpGhost = desk.controlGhost(); // vraci 1, pokud je na policku pred nim duch
		// System.out.println("help " + helpRight + " " + helpForward + " " + helpEdge +
		// " " + helpAround);

		System.out.println("rotate " + helpRightPresence + " " + sampleDistance[0]);
		measure();
		if ((sampleDistance[0] > 0.3) && (helpRightPresence == 0)) {
			markGhost(sampleDistance[0]);
			rotationRight();
			rot = 1;
		}
		helpForward = desk.controlForward(); // bude 1, pokud tam robot uz byl (policko pred nim)

		if (((helpForward == 1) || (helpEdge == 1)) && (helpGhost == 0) && (helpAround == 0) && (rot == 0)) {
			System.out.println("xxxAxxx " + helpForward + " " + helpEdge);
			if (motorSmall.getTachoCount() < -80) {
				motorSmall.rotate(180); // otoci US senzor doleva
			}
			Delay.msDelay(100);
			measure();
			distanceSampler.fetchSample(sampleDistance, 0);
			if (sampleDistance[0] > 0.3) {
				markGhost(sampleDistance[0]);
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
				motorSmall.rotate(180); // otoci US senzor doleva
			}
			Delay.msDelay(100);
			measure();
			distanceSampler.fetchSample(sampleDistance, 0);
			if (sampleDistance[0] > 0.3) {
				markGhost(sampleDistance[0]);
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
				motorSmall.rotate(180); // otoci US sensor doleva
				Delay.msDelay(100);
			}
			motorsForward();

			if ((rotNext == -1) || (rotNext == 3)) { // vpravo
				Delay.msDelay(300);
				distanceSampler.fetchSample(sampleDistance, 0);
				while ((sampleDistance[0] < 0.3) && (sampleTouch[0] == 0)) {
					distanceSampler.fetchSample(sampleDistance, 0);
					touchM.fetchSample(sampleTouch, 0);
				}
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
					if (motorSmall.getTachoCount() < -80) {
						motorSmall.rotate(180);
					}
					Delay.msDelay(100);
				}
			} else if ((rotNext == 1) || (rotNext == -3)) { // vlevo
				Delay.msDelay(300);
				distanceSampler.fetchSample(sampleDistance, 0);
				while ((sampleDistance[0] < 0.3) && (sampleTouch[0] == 0)) {
					distanceSampler.fetchSample(sampleDistance, 0);
					touchM.fetchSample(sampleTouch, 0);
				}
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
	 * resi, co bude robot delat na policku pred duchem
	 */
	private int beforeGhost() {
		int helpGhost;
		int help = 0;
		motorL.resetTachoCount();
		motorsForward();
		Delay.msDelay(50);
		while ((motorL.getTachoCount() < 350) && (sampleTouch[0] == 0) && (sampleDistance[0] < 0.3)) {
			touchM.fetchSample(sampleTouch, 0);
			distanceSampler.fetchSample(sampleDistance, 0);
		}
		if (sampleDistance[0] > 0.3) {
			help = 1;
		} else {
			measure();
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
		// measure();
		// System.out.println("measure " + desk.getRow() + " " + desk.getColumn());
		motorsStop();
		if (motorSmall.getTachoCount() < -80) { // otoci US sensor rovne
			motorSmall.rotate(90);
			Delay.msDelay(100);
		} else if (motorSmall.getTachoCount() > 80) {
			motorSmall.rotate(-90);
			Delay.msDelay(100);
		}
		distanceSampler.fetchSample(sampleDistance, 0);
		markGhost(sampleDistance[0]);
		helpGhost = desk.controlGhost(); // vraci 1, pokud je na policku pred nim duch
	//	System.out.println("helpGhost " + helpGhost + " " + sampleDistance[0]);
		if (helpGhost == 1) {
			motorL.resetTachoCount();
			motorsForward();
			while (motorL.getTachoCount() < (sampleDistance[0] - 0.08) * 2100) {
				distanceSampler.fetchSample(sampleDistance, 0);
			}
			motorsStop();
			desk.measureForward();
			if (motorSmall.getTachoCount() > -70) {
				motorSmall.rotate(-90); // otoci US sensor doprava
				Delay.msDelay(100);
			}
			measure();
		} else {
			if (motorSmall.getTachoCount() > -70) {
				motorSmall.rotate(-90); // otoci US sensor doprava
				Delay.msDelay(100);
			}
			motorsForward();
			while ((motorL.getTachoCount() < 540) && (sampleTouch[0] == 0) && (sampleDistance[0] < 0.3)) {
				if ((motorL.getTachoCount() > 370) && (motorL.getTachoCount() < 420)) {
					measure();
				}
				touchM.fetchSample(sampleTouch, 0);
				distanceSampler.fetchSample(sampleDistance, 0);
			}
			
			if (help == 0) {
				desk.move();
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
		motorL.resetTachoCount();
		if (motorSmall.getTachoCount() > -70) {
			motorSmall.rotate(-90); // otoci US sensor doprava
			Delay.msDelay(100);
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
		number = (int) ((length) / 0.28) + 1;
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
			while (sampleGyro[0] < 75) {
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
	 * vrati robota na vychozi pozici
	 */
	private void reset() {
		System.out.println("reset");
		motorsStop();
		desk.reset();
		Delay.msDelay(2000);
		goToLabyrinth();
	}
}