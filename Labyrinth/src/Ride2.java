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

public class Ride2 {

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
	double distance;
	int[][] desk = new int[6][9];

	float[] lastRange;
	float[] sampleTouch;
	float[] sampleGyro;

	public Ride2() {
		motorL = new EV3LargeRegulatedMotor(MotorPort.C);
		motorR = new EV3LargeRegulatedMotor(MotorPort.B);
		motorSmall = new EV3LargeRegulatedMotor(MotorPort.A);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		touch = new EV3TouchSensor(SensorPort.S2);
		distanceSensor = new EV3UltrasonicSensor(SensorPort.S3);
		gyro = new EV3GyroSensor(SensorPort.S4);
		distanceSampler = distanceSensor.getDistanceMode();
		lastRange = new float[distanceSampler.sampleSize()];
		touchM = touch.getTouchMode();
		sampleTouch = new float[touchM.sampleSize()];
		gyroSampler = gyro.getAngleAndRateMode();
		sampleGyro = new float[gyroSampler.sampleSize()];

		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 9; j++) {
				desk[i][j] = 3;
			}
		}

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
		distanceSampler.fetchSample(lastRange, 0);
		distance = lastRange[0];
		start();

	}

	private void start() {
		oneStep();
		rotationRight();
		motorSmall.rotate(90); // ultrazvukový senzor otoèí doleva
		measureLeft();

		oneEdge(2);
		oneEdge(2);
		oneEdge(4);
		oneEdge(2);

		oneStepL();
	}

	private void oneEdge(int reps) {
		for (int i = 0; i < reps; i++) {
			oneStepL();
		}
		rotationRight();
		measureLeft();
	}

	private void measureLeft() {
		distanceSampler.fetchSample(lastRange, 0);
		if ((lastRange[0] < 0.2) && (lastRange[0] > 0)) {
			switch (direct) {
			case 0:
				desk[row][column - 1] = 2;
				break;
			case 1:
				desk[row - 1][column] = 2;
				break;
			case 2:
				desk[row][column + 1] = 2;
				break;
			case 3:
				desk[row + 1][column] = 2;
				break;
			}
		}
	}

	private void measureRight() {
		distanceSampler.fetchSample(lastRange, 0);
		if ((lastRange[0] < 0.2) && (lastRange[0] > 0)) {
			switch (direct) {
			case 0:
				desk[row][column + 1] = 2;
				break;
			case 1:
				desk[row + 1][column] = 2;
				break;
			case 2:
				desk[row][column - 1] = 2;
				break;
			case 3:
				desk[row - 1][column] = 2;
				break;
			}
		}
	}

	private void oneStep() {
		System.out.println("delam jeden krok");
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

	private void oneStepL() {
		oneStep();
		measureLeft();
	}

	private void oneStepRotate() {
		motorL.startSynchronization();
		motorL.rotate(573);
		motorR.rotate(573);
		motorL.endSynchronization();
		Delay.msDelay(1900);
	}

	// když bude napravo volno, vrátí 0
	// když narazí, vrátí 1
	private int rideWhileSomething() {
		int steps = 0;
		int ret;
		motorL.resetTachoCount();
		motorsForward();
		do {
			distanceSampler.fetchSample(lastRange, 0);
			touchM.fetchSample(sampleTouch, 0);
		} while ((lastRange[0] < 0.2) && (sampleTouch[0] == 0));
		motorsStop();

		if (lastRange[0] < 0.2) {
			ret = 0;
		} else {
			ret = 1;
		}

		steps = (int) (((motorL.getTachoCount() * 17.593) / 360) / 0.28);
		// o = 3.14159 * 5.6 = 17.593
		// 360° = 17.593 cm
		// n° = x cm
		// x = n° * 17.593 / 360°

		switch (direct) {
		case 0:
			row -= steps;
			break;
		case 1:
			column += steps;
			break;
		case 2:
			row += steps;
			break;
		case 3:
			column -= steps;
			break;
		}

		return ret;
	}

	private void motorsForward() {
		motorL.startSynchronization();
		motorL.forward();
		motorR.forward();
		motorL.endSynchronization();
	}

	private void motorsStop() {
		motorL.startSynchronization();
		motorL.stop();
		motorR.stop();
		motorL.endSynchronization();
	}

	/*
	private void rotationLeft() {
		System.out.println("tocim doleva");
		motorL.startSynchronization();
		motorR.rotate(180);
		motorL.rotate(-180);
		motorL.endSynchronization();
		Delay.msDelay(1200);

		if (direct == 0) {
			direct = 3;
		} else {
			direct--;
		}
	}
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
		
		motorL.startSynchronization();
		motorL.stop();
		motorR.stop();
		motorL.endSynchronization();
		gyro.reset();
		Delay.msDelay(500);
		
		if (direct == 0) {
			direct = 3;
		} else {
			direct--;
		}
	}
	
	private void rotationRight() {
		gyro.reset();
		gyroSampler.fetchSample(sampleGyro, 0);
		motorL.startSynchronization();
		motorL.forward();
		motorR.backward();
		motorL.endSynchronization();

		while (sampleGyro[0] > -74) {
			gyroSampler.fetchSample(sampleGyro, 0);
		}
		
		motorL.startSynchronization();
		motorL.stop();
		motorR.stop();
		motorL.endSynchronization();
		gyro.reset();
		Delay.msDelay(500);
		
		if (direct == 3) {
			direct = 0;
		} else {
			direct++;
		}
	}

	/*
	private void rotationRight() {
		System.out.println("tocim doprava");
		motorL.startSynchronization();
		motorR.rotate(-200);
		motorL.rotate(200);
		motorL.endSynchronization();
		Delay.msDelay(1300);

		if (direct == 3) {
			direct = 0;
		} else {
			direct++;
		}
	}
*/
	
	private void rotation180() {
		motorL.startSynchronization();
		motorR.rotate(360);
		motorL.rotate(-360);
		motorL.endSynchronization();
		Delay.msDelay(1200);

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