import java.util.LinkedList;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class Desk {

	RegulatedMotor motorL;
	RegulatedMotor motorR;

	int row;
	int column;
	int direct;
	int[][] desk = new int[6][9];
	int[][] deskHelp = new int[6][9];
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
				deskHelp[i][j] = 3;
			}
		}
		desk[3][3] = 2; // startovni pole
		desk[3][4] = 2; // startovni pole
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
		desk[row][column] = 0;
		System.out.println("move " + row + " " + column);
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
	 * vraci 1, pokud je na policku pred nim duch
	 */
	public int controlGhost() {
		int help = 0;
		switch (direct) {
		case 0:
			if ((row > 0) && ((desk[row - 1][column] == 4))) {
				help = 1;
			}
			break;
		case 1:
			if ((column < 8) && ((desk[row][column + 1] == 4))) {
				help = 1;
			}
			break;
		case 2:
			if ((row < 5) && ((desk[row + 1][column] == 4))) {
				help = 1;
			}
			break;
		case 3:
			if ((column > 0) && ((desk[row][column - 1] == 4))) {
				help = 1;
			}
			break;
		}
		return help;
	}
	
	/*
	 * vraci 1, pokud je na policku vpravo od nej duch
	 */
	public int controlGhostRight() {
		int help = 0;
		switch (direct) {
		case 0:
			if ((column < 8) && (desk[row][column + 1] == 4)) {
				help = 1;
			}
			break;
		case 1:
			if ((row < 5) && (desk[row + 1][column] == 4)) {
				help = 1;
			}
			break;
		case 2:
			if ((column > 0) && (desk[row][column - 1] == 4)) {
				help = 1;
			}
			break;
		case 3:
			if ((row > 0) && (desk[row - 1][column] == 4)) {
				help = 1;
			}
			break;	
		}
		return help;
	}

	/*
	 * vraci 1, pokud je pred robotem okraj bludiste
	 */
	public int controlEdge() {
		int help = 0;
		switch (direct) {
		case 0:
			if (row == 0) {
				help = 1;
			}
			break;
		case 1:
			if (column == 8) {
				help = 1;
			}
			break;
		case 2:
			if (row == 5) {
				help = 1;
			}
			break;
		case 3:
			if (column == 0) {
				help = 1;
			}
			break;
		}
		return help;
	}

	/*
	 * vrací 1, pokud tam robot uz byl nebo je tam prekazka
	 */
	public int controlPresenceRight() {
		int help = 0;
		switch (direct) {
		case 0:
			if ((column < 8) && ((desk[row][column + 1] == 0) || (desk[row][column + 1] == 2))) {
				help = 1;
			}
			break;
		case 1:
			if ((row < 5) && ((desk[row + 1][column] == 0) || (desk[row + 1][column] == 2))) {
				help = 1;
			}
			break;
		case 2:
			if ((column > 0) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2))) {
				help = 1;
			}
			break;
		case 3:
			if ((row > 0) && ((desk[row - 1][column] == 0) || (desk[row - 1][column] == 2))) {
				help = 1;
			}
			break;
		}
		return help;
	}

	/*
	 * vraci 1, pokud ma robot na prave strane hranu bludiste
	 */
	public int controlPresenceEdge() {
		int help = 0;
		switch (direct) {
		case 0:
			if (column == 8) {
				help = 1;
			}
			break;
		case 1:
			if (row == 5) {
				help = 1;
			}
			break;
		case 2:
			if (column == 0) {
				help = 1;
			}
			break;
		case 3:
			if (row == 0) {
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
			if ((row == 0) || ((row > 0) && ((desk[row - 1][column] == 0) || (desk[row - 1][column] == 2)))) { // pred
																												// robotem
				if ((row == 5) || ((row < 5) && ((desk[row + 1][column] == 0) || (desk[row + 1][column] == 2)))) { // za
																													// robotem
					if ((column == 0)
							|| ((column > 0) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2)))) { // nalevo
																													// od
						// robota
						// System.out.println("presence 0");
						help = 1;
					}
				}
			}
			break;
		case 1:
			if ((column == 8) || ((column < 8) && ((desk[row][column + 1] == 0) || (desk[row][column + 1] == 2)))) { // pred
																														// robotem
				if ((column == 0) || ((column > 0) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2)))) { // za
																															// robotem
					if ((row == 0) || ((row > 0) && ((desk[row - 1][column] == 0) || (desk[row - 1][column] == 2)))) { // nalevo
																														// od
						// robota
						// System.out.println("presence 1");
						help = 1;
					}
				}
			}
			break;
		case 2:
			if ((row == 5) || ((row < 5) && ((desk[row + 1][column] == 0) || (desk[row + 1][column] == 2)))) { // pred
																												// robotem
				if ((row == 0) || ((row > 0) && ((desk[row - 1][column] == 0) || (desk[row - 1][column] == 2)))) { // za
																													// robotem
					if ((column == 8)
							|| ((column < 8) && ((desk[row][column + 1] == 0) || (desk[row][column + 1] == 2)))) { // nalevo
																													// od
						// robota
						// System.out.println("coordinate " + row + " " + column + " " + direct);
						// System.out.println("presence 2");
						help = 1;
					}
				}
			}
			break;
		case 3:
			if ((column == 0) || ((column > 0) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2)))) { // pred
																														// robotem
				if ((column == 8) || ((column < 8) && ((desk[row][column + 1] == 0) || (desk[row][column + 1] == 2)))) { // za
																															// robotem
					if ((row == 5) || ((row < 5) && ((desk[row + 1][column] == 0) || (desk[row + 1][column] == 2)))) { // nalevo
																														// od
						// robota
						// System.out.println("presence 3");
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
				System.out.println("measureF " + (row - 1) + " " + column);
			}
			break;
		case 1:
			if ((column < 8) && (desk[row][column + 1] != 0)) {
				desk[row][column + 1] = 2;
				System.out.println("measureF " + row + " " + (column + 1));
			}
			break;
		case 2:
			if ((row < 5) && (desk[row + 1][column] != 0)) {
				desk[row + 1][column] = 2;
				System.out.println("measureF " + (row + 1) + " " + column);
			}
			break;
		case 3:
			if ((column > 0) && (desk[row][column - 1] != 0)) {
				desk[row][column - 1] = 2;
				System.out.println("measureF " + row + " " + (column - 1));
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
					// System.out.println("measureL " + row + " " + (column - 1));
				}
				break;
			case 1:
				if ((row > 0) && (desk[row - 1][column] != 0)) {
					desk[row - 1][column] = 2;
					// System.out.println("measureL " + (row - 1) + " " + column);
				}
				break;
			case 2:
				if ((column < 8) && (desk[row][column + 1] != 0)) {
					desk[row][column + 1] = 2;
					// System.out.println("measureL " + row + " " + (column + 1));
				}
				break;
			case 3:
				if ((row < 5) && (desk[row + 1][column] != 0)) {
					desk[row + 1][column] = 2;
					// System.out.println("measureL " + (row + 1) + " " + column);
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
					// System.out.println("measureR " + row + " " + (column + 1));
				}
				break;
			case 1:
				if ((row < 5) && (desk[row + 1][column] != 0)) {
					desk[row + 1][column] = 2;
					// System.out.println("measureR " + (row + 1) + " " + column);
				}
				break;
			case 2:
				if ((column > 0) && (desk[row][column - 1] != 0)) {
					desk[row][column - 1] = 2;
					// System.out.println("measureR " + row + " " + (column - 1));
				}
				break;
			case 3:
				if ((row > 0) && (desk[row - 1][column] != 0)) {
					desk[row - 1][column] = 2;
					// System.out.println("measureR " + (row - 1) + " " + column);
				}
				break;
			}
		}
	}
	
	/*
	 * oznaci ducha pred robotem 
	 
	public void measureGhost() {
		switch (direct) {
		case 0:
			if ((row > 0) && (desk[row - 1][column] != 0)) {
				desk[row - 1][column] = 5;
				System.out.println("duch uricte " + (row - 1) + " " + column);
			}
			break;
		case 1:
			if ((column < 8) && (desk[row][column + 1] != 0)) {
				desk[row][column + 1] = 5;
				System.out.println("duch urcite " + row + " " + (column + 1));
			}
			break;
		case 2:
			if ((row < 5) && (desk[row + 1][column] != 0)) {
				desk[row + 1][column] = 5;
				System.out.println("duch urcite " + (row + 1) + " " + column);
			}
			break;
		case 3:
			if ((column > 0) && (desk[row][column - 1] != 0)) {
				desk[row][column - 1] = 5;
				System.out.println("duch urcite " + row + " " + (column - 1));
			}
			break;
		}
	}
*/
	
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
				desk[row][column] = 2;
				row++;
			}
			break;
		case 1:
			if ((column < 8) && (column > 0)) {
				desk[row][column] = 2;
				column--;
			}
			break;
		case 2:
			if ((row < 5) && (row > 0)) {
				desk[row][column] = 2;
				row--;
			}
			break;
		case 3:
			if ((column < 8) && (column > 0)) {
				desk[row][column] = 2;
				column++;
			}
			break;
		}
		System.out.println("back " + row + " " + column);
	}
	
	public void resetField() {
		switch (direct) {
		case 0:
			if (row > 0) {
				desk[row - 1][column] = deskHelp[row - 1][column];
			}
			break;
		case 1:
			if (column < 8) {
				desk[row][column + 1] = deskHelp[row][column + 1];
			}
			break;
		case 2:
			if (row < 5) {
				desk[row + 1][column] = deskHelp[row + 1][column];
			}
			break;
		case 3:
			if (column > 0) {
				desk[row][column - 1] = deskHelp[row][column - 1];
			}
			break;
		}
		desk[row][column] = 0;
	}
	
	public void resetField(int row, int column) {
		switch (direct) {
		case 0:
			if (row > 0) {
				desk[row][column] = deskHelp[row][column];
				System.out.println("desk " + (row) + " " + column + " " + desk[row][column]);
			}
			break;
		case 1:
			if (column < 8) {
				desk[row][column] = deskHelp[row][column];
				System.out.println("desk " + (row) + " " + (column) + " " + desk[row][column]);
			}
			break;
		case 2:
			if (row < 5) {
				desk[row][column] = deskHelp[row][column];
				System.out.println("desk " + (row) + " " + column + " " + desk[row][column]);
			}
			break;
		case 3:
			if (column > 0) {
				desk[row][column] = deskHelp[row][column];
				System.out.println("desk " + (row) + " " + (column) + " " + desk[row][column]);
			}
			break;
		}
		desk[row][column] = 0;
	}

	public void rotationL() {
		if (direct == 0) {
			direct = 3;
		} else {
			direct--;
		}
		System.out.println("rot left " + row + " " + column + " " + direct);
	}

	public void rotationR() {
		if (direct == 3) {
			direct = 0;
		} else {
			direct++;
		}
		System.out.println("rot right " + row + " " + column + " " + direct);
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
		System.out.println("rot 180 " + row + " " + column + " " + direct);
	}

	public int findWay() {
		BFS.inicialize(desk, row, column, direct);
		way = BFS.getWay(); // vrati cestu od zacatku do konce (bez aktuálního pole)
		return way.size();
	}

	/*
	 * vrati smer, kterym se vydat k neprozkoumanemu policku
	 */
	public int getRotation() {
		int ret = -5;
		if (way.size() > 0) {
			int rowHelp = (way.getFirst() / 10) - 1;
			int columnHelp = way.getFirst() % 10;
			int directHelp = 0;
			int rowH = row;
			int columnH = column;
			
			switch (direct) {
			case 0:
					rowH--;
				break;
			case 1:
					columnH++;
				break;
			case 2:
					rowH++;
				break;
			case 3:
					columnH--;
				break;
			}

			if (rowHelp - rowH == -1) {
				directHelp = 0;
			} else if (rowHelp - rowH == 1) {
				directHelp = 2;
			}

			if (columnHelp - columnH == -1) {
				directHelp = 3;
			} else if (columnHelp - columnH == 1) {
				directHelp = 1;
			}
			ret = direct - directHelp;
		}
		return ret;
	}

	/*
	 * vrati smer, kterym se vydat k neprozkoumanemu policku a smaze prvni policko
	 */
	public int rotationToWay() {
		int rowHelp = (way.getFirst() / 10) - 1;
		int columnHelp = way.getFirst() % 10;
		int directHelp = 0;
		int ret = 0;

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
		ret = direct - directHelp;

		way.removeFirst();
		return ret;
	}

	public void markGhost(int number, int directHelp) {
		int directH;
		directH = direct + directHelp;
		if (directH == 4) {
			directH = 0;
		} else if (directH == -1) {
			directH = 3;
		}
		switch (directH) {
		case 0:
			if ((row - number >= 0) && (column > 0) && (column < 9) && (desk[row - number][column] != 0)  && (desk[row - number][column] != 2)) {
				System.out.println("duch na " + (row - number) + " " + column);
				deskHelp[row - number][column] = desk[row - number][column];
				desk[row - number][column] = 4;
			}
			if (number > 1) {
				for (int i = 0; i < number - 1; i++) {
					if ((row - i - 1 > 0) && (desk[row - i - 1][column] == 4)) {
						resetField(row - i - 1, column);
					}
				}
			}
			break;
		case 1:
			if ((column + number < 9) && (row > 0) && (row < 6) && (desk[row][column + number] != 0) && (desk[row][column + number] != 2)) {
				System.out.println("duch na " + row + " " + (column + number));
				deskHelp[row][column + number] = desk[row][column + number];
				desk[row][column + number] = 4;
			}
			if (number > 1) {
				for (int i = 0; i < number - 1; i++) {
					if ((column + i + 1 < 9) && (desk[row][column + i + 1] == 4)) {
						resetField(row, column + i + 1);
					}
				}
			}
			break;
		case 2:
			if ((row + number < 6) && (column > 0) && (column < 9) && (desk[row + number][column] != 0) && (desk[row + number][column] != 2)) {
				System.out.println("duch na " + (row + number) + " " + column);
				deskHelp[row + number][column] = desk[row + number][column];
				desk[row + number][column] = 4;
			}
			if (number > 1) {
				for (int i = 0; i < number - 1; i++) {
					if ((row + i + 1 < 6) && (desk[row + i + 1][column] == 4)) {
						resetField(row + i + 1, column);
					}
				}
			}
			break;
		case 3:
			if ((column - number >= 0) && (row > 0) && (row < 6) && (desk[row][column - number] != 0) && (desk[row][column - number] != 2)) {
				System.out.println("duch na " + row + " " + (column - number));
				deskHelp[row][column - number] = desk[row][column - number];
				desk[row][column - number] = 4;
			}
			if (number > 1) {
				for (int i = 0; i < number - 1; i++) {
					if ((column - i - 1 > 0) && (desk[row][column - i - 1] == 4)) {
						resetField(row, column - i - 1);
					}
				}
			}
			break;
		}
	}
	
	public void eraseWay() {
		way.clear();
		BFS.stop();
	}
	
	public void reset() {
		row = 3;
		column = 4;
		direct = 0;
	}

	/*
	 * public void initialCondition() { // row = 0; // column = 8; // direct = 1;
	 * desk[0][3] = 0; desk[0][4] = 0; desk[0][5] = 0; desk[0][6] = 0; desk[0][7] =
	 * 0; desk[0][8] = 0;
	 * 
	 * desk[1][3] = 0; desk[1][4] = 2; desk[1][5] = 0; // desk[1][6] = 2; desk[1][7]
	 * = 0; desk[1][8] = 2;
	 * 
	 * desk[2][2] = 0; desk[2][3] = 0; // desk[2][4] = 0; desk[2][5] = 0; desk[2][6]
	 * = 0; desk[2][7] = 2;
	 * 
	 * desk[3][2] = 0; desk[3][3] = 2; desk[3][4] = 2; desk[3][5] = 2; desk[3][6] =
	 * 0;
	 * 
	 * desk[4][2] = 0; desk[4][3] = 0; desk[4][4] = 0; desk[4][5] = 0; desk[4][6] =
	 * 0; }
	 */

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
