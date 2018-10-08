import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Desk {

	int row;
	int column;
	int direct;
	int[][] desk = new int[6][9];
	
	EV3UltrasonicSensor distanceSensor;
	RegulatedMotor motorL;
	RegulatedMotor motorR;
	SampleProvider distanceSampler;
	float[] sampleDistance; 
	
	public Desk(EV3UltrasonicSensor distanceSensor) {
		this.distanceSensor = distanceSensor;
		distanceSampler = distanceSensor.getDistanceMode();
		sampleDistance = new float[distanceSampler.sampleSize()];
		
		row = 3;
		column = 4;
		direct = 0;

		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 9; j++) {
				desk[i][j] = 3;
			}
		}
		desk[3][3] = 2; // startovni pole
		desk[3][4] = 0; // startovni pole
		desk[3][5] = 2; // startovni pole
	}

	/*
	 * presune robota v souradnicovem systemu
	 */
	public void move() {
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
	public int control() {
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
	public int controlPresenceRight() {
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
	public void measureLeft() {
		System.out.println("vlevo " + sampleDistance[0]);
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
	public void measureRight() {
		distanceSampler.fetchSample(sampleDistance, 0);
		System.out.println("vpravo " + sampleDistance[0]);
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
	public void oneStep() {
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

	public void rotationL() {
		if (direct == 0) {
			direct = 3;
		} else {
			direct--;
		}
	}

	public void rotationR() {
		if (direct == 3) {
			direct = 0;
		} else {
			direct++;
		}
	}
	
	public void rotationB() {
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

	public void turnOffLight() {
		desk[row][column] = 0;
	}

	public int getState(int row, int column) {
		return desk[row][column];
	}
	
	public int getRow() {
		return row;
	}

	public int getColumn() {
		return column;
	}
}
