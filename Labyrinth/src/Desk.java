import java.util.LinkedList;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Desk {

	RegulatedMotor motorL;
	RegulatedMotor motorR;

	int row;
	int column;
	int direct;
	int[][] desk = new int[6][9];
	BFS BFS;
	LinkedList<Integer> way = new LinkedList<Integer>();

	EV3UltrasonicSensor distanceSensor;
	SampleProvider distanceSampler;
	float[] sampleDistance;

	public Desk(EV3UltrasonicSensor distanceSensor) {
		this.distanceSensor = distanceSensor;
		distanceSampler = distanceSensor.getDistanceMode();
		sampleDistance = new float[distanceSampler.sampleSize()];
		BFS = new BFS();

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
			if ((column > 0) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2))) {
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

	public int controlPresenceAround() {
		int help = 0;
		switch (direct) {
		case 0:
			if ((row > 0) && ((desk[row - 1][column] == 0) || (desk[row - 1][column] == 2))) { // pred robotem
				if ((row < 5) && ((desk[row + 1][column] == 0) || (desk[row + 1][column] == 2))) { // za robotem
					if ((column > 0) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2))) { // nalevo od
																											// robota
						help = 1;
					}
				}
			}
			break;
		case 1:
			if ((column < 8) && ((desk[row][column + 1] == 0) || (desk[row][column + 1] == 2))) { // pred robotem
				if ((column > 0) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2))) { // za robotem
					if ((row > 0) && ((desk[row - 1][column] == 0) || (desk[row - 1][column] == 2))) { // nalevo od
																										// robota
						help = 1;
					}
				}
			}
			break;
		case 2:
			if ((row < 5) && ((desk[row + 1][column] == 0) || (desk[row + 1][column] == 2))) { // pred robotem
				if ((row > 0) && ((desk[row - 1][column] == 0) || (desk[row - 1][column] == 2))) { // za robotem
					if ((column < 8) && ((desk[row][column + 1] == 0) || (desk[row][column + 1] == 2))) { // nalevo od
																											// robota
						help = 1;
					}
				}
			}
			break;
		case 3:
			if ((column > 0) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2))) { // pred robotem
				if ((column < 8) && ((desk[row][column + 1] == 0) || (desk[row][column + 1] == 2))) { // za robotem
					if ((row < 5) && ((desk[row + 1][column] == 0) || (desk[row + 1][column] == 2))) { // nalevo od
																										// robota
						help = 1;
					}
				}
			}
			break;
		}

		return help;
	}

	/*
	 * mapuje prekazky pred robotem po sepnuti dotykoveho senzoru
	 */
	public void measureForward() {
		switch (direct) {
		case 0:
			if ((row > 0) && (desk[row - 1][column] != 0)) {
				desk[row - 1][column] = 2;
				System.out.println("measure " + (row - 1) + " " + column);
			}
			break;
		case 1:
			if ((column < 8) && (desk[row][column + 1] != 0)) {
				desk[row][column + 1] = 2;
				System.out.println("measure " + row + " " + (column + 1));
			}
			break;
		case 2:
			if ((row < 5) && (desk[row + 1][column] != 0)) {
				desk[row + 1][column] = 2;
				System.out.println("measure " + (row + 1) + " " + column);
			}
			break;
		case 3:
			if ((column > 0) && (desk[row][column - 1] != 0)) {
				desk[row][column - 1] = 2;
				System.out.println("measure " + row + " " + (column - 1));
			}
			break;
		}
	}

	/*
	 * mapuje prekazky nalevo od robota
	 */
	public void measureLeft() {
		distanceSampler.fetchSample(sampleDistance, 0);
		if ((sampleDistance[0] < 0.2) && (sampleDistance[0] > 0)) {
			switch (direct) {
			case 0:
				if ((column > 0) && (desk[row][column - 1] != 0)) {
					desk[row][column - 1] = 2;
					System.out.println("measure " + row + " " + (column - 1));
				}
				break;
			case 1:
				if ((row > 0) && (desk[row - 1][column] != 0)) {
					desk[row - 1][column] = 2;
					System.out.println("measure " + (row - 1) + " " + column);
				}
				break;
			case 2:
				if ((column < 8) && (desk[row][column + 1] != 0)) {
					desk[row][column + 1] = 2;
					System.out.println("measure " + row + " " + (column + 1));
				}
				break;
			case 3:
				if ((row < 5) && (desk[row + 1][column] != 0)) {
					desk[row + 1][column] = 2;
					System.out.println("measure " + (row + 1) + " " + column);
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
		if ((sampleDistance[0] < 0.2) && (sampleDistance[0] > 0)) {
			switch (direct) {
			case 0:
				if ((column < 8) && (desk[row][column + 1] != 0)) {
					desk[row][column + 1] = 2;
					System.out.println("measure " + row + " " + column + 1);
				}
				break;
			case 1:
				if ((row < 5) && (desk[row + 1][column] != 0)) {
					desk[row + 1][column] = 2;
				}
				break;
			case 2:
				if ((column > 0) && (desk[row][column - 1] != 0)) {
					desk[row][column - 1] = 2;
				}
				break;
			case 3:
				if ((row > 0) && (desk[row - 1][column] != 0)) {
					desk[row - 1][column] = 2;
				}
				break;
			}
		}
	}

	public void oneStepMove() {
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
		desk[row][column] = 0;
	}
	
	public void back() {
		switch (direct) {
		case 0:
			if ((row < 5) && (row > 0)) {
				row++;
			}
			break;
		case 1:
			if ((column < 8) && (column > 0)) {
				column--;
			}
			break;
		case 2:
			if ((row < 5) && (row > 0)) {
				row--;
			}
			break;
		case 3:
			if ((column < 8) && (column > 0)) {
				column++;
			}
			break;
		}
		System.out.println("back " + row + " " + column);
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

	public int rotationToWay() {
		int rowHelp = (way.getFirst() / 10) - 1;
		int columnHelp = way.getFirst() % 10;
		int directHelp = 0;

		if (rowHelp - row == -1) {
			directHelp = 0;
		} else if (rowHelp - row == 1) {
			directHelp = 2;
		}

		if (columnHelp - column == -1) {
			directHelp = 3;
		} else if (columnHelp - column == 1) {
			directHelp = 1;
		}

		switch (direct - directHelp) {
		case -3:
			rotationL();
			break;
		case -2:
			rotationB();
			break;
		case -1:
			rotationR();
			break;
		case 1:
			rotationL();
			break;
		case 2:
			rotationB();
			break;
		case 3:
			rotationR();
			break;
		}
		way.removeFirst();
		return direct - directHelp;
	}

	public void turnOffLight() {
		desk[row][column] = 0;
	}

	public LinkedList<Integer> findWay() {
		BFS.inicialize(desk, row, column, direct);
		way = BFS.getWay(); // vrati cestu od zacatku do konce (bez aktuálního pole)
		return way;
	}
	
	public void eraseWay() {
		BFS.stop();
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
