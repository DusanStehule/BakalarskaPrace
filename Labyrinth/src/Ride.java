import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Ride {

	RegulatedMotor motorL;
	RegulatedMotor motorR;
	RegulatedMotor motorSmall;

	EV3UltrasonicSensor distanceSensor;
	SampleProvider distanceSampler;
	SampleProvider listenSampler;

	int row;
	int column;
	int direct;
	double distance;
	int[][] desk = new int[6][9];

	float[] lastRange;

	public Ride() {
		motorL = new EV3LargeRegulatedMotor(MotorPort.C);
		motorR = new EV3LargeRegulatedMotor(MotorPort.B);
		motorSmall = new EV3LargeRegulatedMotor(MotorPort.A);
		motorL.synchronizeWith(new RegulatedMotor[] { motorR });
		distanceSensor = new EV3UltrasonicSensor(SensorPort.S3);
		distanceSampler = distanceSensor.getDistanceMode();
		lastRange = new float[distanceSampler.sampleSize()];

		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 9; j++) {
				desk[i][j] = 3;
			}
		}

		row = 3;
		column = 4;
		direct = 0;
		desk[row][column] = 0;
		start();
		motorL.close();
		motorR.close();
		distanceSensor.close();
	}

	private void start() {
		distanceSampler.fetchSample(lastRange, 0);
		distance = lastRange[0];
		int steps = (int) (distance / 0.28);
		System.out.println("nameril jsem " + distance + "m a udelam " + steps + " kroku");
		// natoèí ultrazvukový senzor doprava
		motorSmall.rotate(-90);
		Delay.msDelay(3000);

		for (int i = 0; i < steps; i++) {
			System.out.println("jedu cyklus");
			oneStep();
		}

		System.out.println("ted jsem za cyklem");
		// natoèí ultrazvukový senzor doleva
		motorSmall.rotate(180);
		distanceSampler.fetchSample(lastRange, 0);
		distance = lastRange[0];
		steps = (int) (distance / 0.28);
		// natoèí ultrazvukový senzor rovnì
		motorSmall.rotate(-90);

		if (steps > 0) {
			rotationLeft();
			start();
		} else {
			rotationRight();
			start();
		}

	}

	private void oneStep() {
		System.out.println("delam jeden krok");
		switch (direct) {
		case 0:
			if (desk[row - 1][column] != 0) {
				oneStepRotate();
				row--;
			}
			break;
		case 1:
			if (desk[row][column + 1] != 0) {
				oneStepRotate();
				column++;
			}
			break;
		case 2:
			if (desk[row + 1][column] != 0) {
				oneStepRotate();
				row++;
			}
			break;
		case 3:
			if (desk[row][column - 1] != 0) {
				oneStepRotate();
				column--;
			}
			break;
		}

		desk[row][column] = 0;
		measureRight();
	}

	private void oneStepRotate() {
		System.out.println("tocim jeden krok");
		motorL.startSynchronization();
		motorL.rotate(573);
		motorR.rotate(573);
		motorL.endSynchronization();
		Delay.msDelay(2240);
	}

	private void measureRight() {
		System.out.println("merim vpravo");
		int columnHelp = column;
		int rowHelp = row;
		int help = 0;
		int numberOfSteps;
		distanceSampler.fetchSample(lastRange, 0);
		distance = lastRange[0];
		numberOfSteps = (int) (distance / 0.28);

		switch (direct) {
		case 0:
			for (int i = 0; i < numberOfSteps; i++) {
				if ((columnHelp < 8) && (desk[rowHelp][columnHelp + 1] != 0)) {
					desk[rowHelp][columnHelp] = 1;
				}
				columnHelp++;
				help++;
			}

			System.out.println("po 1. podmince");

			if ((help == 1) && (desk[row + 1][column + 1] == 2) && (desk[row - 1][column + 1] == 2)) {
				oneSquare();
				desk[row][column + 1] = 0;
			}

			System.out.println("po 2. podmince");

			if (column + numberOfSteps < 8) {
				desk[row][column + numberOfSteps + 1] = 2;
			}

			System.out.println("po 3. podmince");
			break;
		case 1:
			for (int i = 0; i < numberOfSteps; i++) {
				if ((rowHelp < 5) && (desk[rowHelp + 1][columnHelp] != 0)) {
					desk[rowHelp][columnHelp] = 1;
				}
				rowHelp++;
				help++;
			}

			if ((help == 1) && (desk[row + 1][column + 1] == 2) && (desk[row + 1][column - 1] == 2)) {
				oneSquare();
				desk[row + 1][column] = 0;
			}

			if (row + numberOfSteps < 5) {
				desk[row + numberOfSteps + 1][column] = 2;
			}

			break;
		case 2:
			for (int i = 0; i < numberOfSteps; i++) {
				if ((columnHelp > 0) && (desk[rowHelp][columnHelp - 1] != 0)) {
					desk[rowHelp][columnHelp] = 1;
				}
				columnHelp--;
				help++;
			}

			if ((help == 1) && (desk[row + 1][column - 1] == 2) && (desk[row - 1][column - 1] == 2)) {
				oneSquare();
				desk[row][column - 1] = 0;
			}

			if (column - numberOfSteps > 0) {
				desk[row][column - numberOfSteps - 1] = 2;
			}

			break;
		case 3:
			for (int i = 0; i < numberOfSteps; i++) {
				if ((rowHelp > 0) && (desk[rowHelp - 1][columnHelp] != 0)) {
					desk[rowHelp][columnHelp] = 1;
				}
				rowHelp--;
				help++;
			}

			if ((help == 1) && (desk[row - 1][column + 1] == 2) && (desk[row - 1][column - 1] == 2)) {
				oneSquare();
				desk[row - 1][column] = 0;
			}

			if (row - numberOfSteps > 0) {
				desk[row - numberOfSteps - 1][column] = 2;
			}

			break;
		}
	}

	private void oneSquare() {
		System.out.println("jeden ètverec");
		rotationRight();
		oneStepRotate();
		rotation180();
		oneStepRotate();
		rotationRight();
	}

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

	private void rotationRight() {
		System.out.println("tocim doprava");
		motorL.startSynchronization();
		motorR.rotate(-180);
		motorL.rotate(180);
		motorL.endSynchronization();
		Delay.msDelay(1200);

		if (direct == 3) {
			direct = 0;
		} else {
			direct++;
		}
	}

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
