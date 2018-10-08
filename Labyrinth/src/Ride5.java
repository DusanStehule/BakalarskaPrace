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

public class Ride5 {

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

	int row;
	int column;
	int direct;
	double lastDistance;
	int[][] desk = new int[6][9];

	float[] sampleDistance;
	float[] sampleTouch;
	float[] sampleGyro;

	public Ride5() {
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

		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 9; j++) {
				desk[i][j] = 3;
			}
		}
		desk[3][3] = 2; // startovni pole
		desk[3][5] = 2; // startovni pole

		row = 3;
		column = 4;
		direct = 0;
		desk[row][column] = 0;
		goToLabyrinth();
		motorL.close();
		motorR.close();
		distanceSensor.close();
		touch.close();
	}

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
		int angle;
		motorL.resetTachoCount();
		distanceSampler.fetchSample(sampleDistance, 0);
		help = control(); // vraci 0, pokud tam robot jeste nebyl
		if (help == 0) {
			motorsForward();

			if ((sampleDistance[0] > 0.3) && (lastDistance > 0)) {
				angle = (int) (360 * lastDistance / 0.17593 + 60);
				while (motorL.getTachoCount() < angle) {
				}
			} else if (sampleDistance[0] > 0.3) {
				Delay.msDelay(800);
			}
		}

		do {

		} while ((motorL.getTachoCount() < 150) && (help == 0));

		if (help == 0) {
			move();
			measure();
		}

		help = control();
		desk[row][column] = 0;
		motorL.resetTachoCount();
		System.out.println("desk1 " + row + " " + column);

		do {
			if (motorL.getTachoCount() > 550) { // puvodne 573
				measure();
				move();
				help = control();
				desk[row][column] = 0;
				System.out.println("desk2 " + row + " " + column);
				motorL.resetTachoCount();
			}

			lastDistance = sampleDistance[0];
			distanceSampler.fetchSample(sampleDistance, 0);
			touchM.fetchSample(sampleTouch, 0);
		} while ((sampleDistance[0] < 0.3) && (sampleTouch[0] == 0) && (help == 0));

		if ((sampleDistance[0] > 0.3) || (help == 1)) {
			Delay.msDelay(700);
			desk[row][column] = 0;
		}

		motorsStop();
		motorL.resetTachoCount();
	}

	/*
	 * zatoci doprava nebo doleva, kdyz: -to doprava nejde -vpravo uz byl
	 */
	private void rotate() {
		int help;
		int help1;
		int angle = -90 - motorSmall.getTachoCount();
		motorSmall.rotate(angle); // otoci US senzor doprava
		distanceSampler.fetchSample(sampleDistance, 0);
		help = controlPresenceRight(); // bude 1, pokud tam robot jeste nebyl (vpravo)
		help1 = control(); // bude 0, pokud tam robot jeste nebyl (policko pred nim)

		if ((sampleDistance[0] > 0.2) && (help == 1)) {
			rotationRight();
		}

		if ((help1 == 1) || (help == 0)) {
			motorSmall.rotate(180); // otoci US senzor doleva
			Delay.msDelay(500);
			distanceSampler.fetchSample(sampleDistance, 0);
			if (sampleDistance[0] > 0.3) {
				rotationLeft();
			} else {
				rotation180();
			}
		}
	}

	/*
	 * dle natoceni US senzoru meri bud nalevo nebo napravo
	 */
	private void measure() {
		if (motorSmall.getTachoCount() == 90) {
			measureLeft();
		} else if (motorSmall.getTachoCount() == -90) {
			measureRight();
		}
	}

	/*
	 * presune robota v souradnicovem systemu
	 */
	private void move() {
		switch (direct) {
		case 0:
			if (row > 0) {
				row--;
			}
			break;
		case 1:
			if (column < 8) {
				column++;
			}
			break;
		case 2:
			if (row < 5) {
				row++;
			}
			break;
		case 3:
			if (column > 0) {
				column--;
			}
			break;
		}
	}

	/*
	 * vraci 0, pokud tam robot jeste nebyl, jinak 1
	 */
	private int control() {
		int help = 0;
		switch (direct) {
		case 0:
			if ((row > 0) && ((desk[row - 1][column] == 0) || (desk[row - 1][column] == 2))) {
				help = 1;
			}
			break;
		case 1:
			if ((column < 8) && ((desk[row][column + 1] == 0) || (desk[row][column + 1] == 2))) {
				help = 1;
			}
			break;
		case 2:
			if ((row < 5) && ((desk[row + 1][column] == 0) || (desk[row + 1][column] == 2))) {
				help = 1;
			}
			break;
		case 3:
			if ((column < 8) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2))) {
				help = 1;
			}
			break;
		}
		return help;
	}

	/*
	 * vrací 1, pokud tam robot jeste nebyl, jinak 0
	 */
	private int controlPresenceRight() {
		int help = 0;
		switch (direct) {
		case 0:
			if ((column < 8) && (desk[row][column + 1] != 0)) {
				help = 1;
			}
			break;
		case 1:
			if ((row < 5) && ((desk[row + 1][column] != 0))) {
				help = 1;
			}
			break;
		case 2:
			if ((column > 0) && ((desk[row][column - 1] != 0))) {
				help = 1;
			}
			break;
		case 3:
			if ((row > 0) && ((desk[row - 1][column] != 0))) {
				help = 1;
			}
			break;
		}
		return help;
	}

	/*
	 * mapuje prekazky nalevo od robota
	 */
	private void measureLeft() {
		distanceSampler.fetchSample(sampleDistance, 0);
		if ((sampleDistance[0] < 0.2) && (sampleDistance[0] > 0)) {
			switch (direct) {
			case 0:
				if (column > 0) {
					desk[row][column - 1] = 2;
				}
				break;
			case 1:
				if (row > 0) {
					desk[row - 1][column] = 2;
				}
				break;
			case 2:
				if (column < 8) {
					desk[row][column + 1] = 2;
				}
				break;
			case 3:
				if (row < 5) {
					desk[row + 1][column] = 2;
				}
				break;
			}
		}
	}

	/*
	 * mapuje prekazky napravo od robota
	 */
	private void measureRight() {
		distanceSampler.fetchSample(sampleDistance, 0);
		if ((sampleDistance[0] < 0.2) && (sampleDistance[0] > 0)) {
			switch (direct) {
			case 0:
				if (column < 8) {
					desk[row][column + 1] = 2;
				}
				break;
			case 1:
				if (row < 5) {
					desk[row + 1][column] = 2;
				}
				break;
			case 2:
				if (column > 0) {
					desk[row][column - 1] = 2;
				}
				break;
			case 3:
				if (row > 0) {
					desk[row - 1][column] = 2;
				}
				break;
			}
		}
	}

	/*
	 * udela jeden krok otocenim obou motoru o 573°
	 */
	private void oneStep() {
		oneStepRotate();
		switch (direct) {
		case 0:
			row--;
			break;
		case 1:
			column++;
			break;
		case 2:
			row++;
			break;
		case 3:
			column--;
			break;
		}
		desk[row][column] = 0;
	}

	/*
	 * rotuje oba motory o 573°
	 */
	private void oneStepRotate() {
		motorL.startSynchronization();
		motorL.rotate(573);
		motorR.rotate(573);
		motorL.endSynchronization();
		Delay.msDelay(1900);
	}

	/*
	 * spusti oba motory dopredu
	 */
	private void motorsForward() {
		motorL.startSynchronization();
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
	 * zatoci doleva (gyro)
	 */
	private void rotationLeft() {
		gyro.reset();
		gyroSampler.fetchSample(sampleGyro, 0);
		motorL.startSynchronization();
		motorL.backward();
		motorR.forward();
		motorL.endSynchronization();

		while (sampleGyro[0] < 79) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}

		motorsStop();
		gyro.reset();
		Delay.msDelay(500);

		if (direct == 0) {
			direct = 3;
		} else {
			direct--;
		}
	}

	/*
	 * zatoci doprava (gyro)
	 */
	private void rotationRight() {
		gyro.reset();
		gyroSampler.fetchSample(sampleGyro, 0);
		motorL.startSynchronization();
		motorL.forward();
		motorR.backward();
		motorL.endSynchronization();

		while (sampleGyro[0] > -73) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}

		motorsStop();
		gyro.reset();
		Delay.msDelay(500);

		if (direct == 3) {
			direct = 0;
		} else {
			direct++;
		}
	}

	/*
	 * otoci se o 180°
	 */
	private void rotation180() {
		gyro.reset();
		gyroSampler.fetchSample(sampleGyro, 0);
		motorL.startSynchronization();
		motorL.backward();
		motorR.forward();
		motorL.endSynchronization();

		while (sampleGyro[0] < 169) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}

		motorsStop();
		gyro.reset();
		Delay.msDelay(500);

		switch (direct) {
		case 0:
			direct = 2;
			break;
		case 1:
			direct = 3;
			break;
		case 2:
			direct = 0;
			break;
		case 3:
			direct = 1;
			break;
		}
	}
}