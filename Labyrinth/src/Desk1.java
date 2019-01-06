import java.util.LinkedList;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Desk1 {

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

	public Desk1(EV3UltrasonicSensor distanceSensor) {
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
	public int controlForward() {
		int help = 0;
		switch (direct) {
		case 0:
			if ((row > 0) && ((desk[row - 1][column] == 0) || (desk[row - 1][column] == 2))) {
			//	System.out.println("...control " + desk[row - 1][column]);
				help = 1;
			}
			break;
		case 1:
			if ((column < 8) && ((desk[row][column + 1] == 0) || (desk[row][column + 1] == 2))) {
			//	System.out.println("...control " + desk[row][column + 1]);
				help = 1;
			}
			break;
		case 2:
			if ((row < 5) && ((desk[row + 1][column] == 0) || (desk[row + 1][column] == 2))) {
			//	System.out.println("...control " + desk[row + 1][column]);
				help = 1;
			}
			break;
		case 3:
			if ((column > 0) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2))) {
			//	System.out.println("...control " + desk[row][column - 1]);
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
	 * vrac� 1, pokud tam robot uz byl nebo je tam prekazka
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
	 * vrac� 1, pokud tam robot uz byl nebo je tam prekazka
	 */
	public int controlPresenceRightPresence() {
		int help = 0;
		switch (direct) {
		case 0:
			if ((column < 8) && ((desk[row][column + 1] == 0))) {
				help = 1;
			}
			break;
		case 1:
			if ((row < 5) && ((desk[row + 1][column] == 0))) {
				help = 1;
			}
			break;
		case 2:
			if ((column > 0) && ((desk[row][column - 1] == 0))) {
				help = 1;
			}
			break;
		case 3:
			if ((row > 0) && ((desk[row - 1][column] == 0))) {
				help = 1;
			}
			break;
		}
		return help;
	}
	
	/*
	 * vrac� 1, pokud tam robot uz byl nebo je tam prekazka
	 */
	public int controlPresenceLeftPresence() {
		int help = 0;
		switch (direct) {
		case 0:
			if ((column > 0) && ((desk[row][column - 1] == 0))) {
				help = 1;
			}
			break;
		case 1:
			if ((row > 0) && ((desk[row - 1][column] == 0))) {
				help = 1;
			}
			break;
		case 2:
			if ((column < 8) && ((desk[row][column + 1] == 0))) {
				help = 1;
			}
			break;
		case 3:
			if ((row < 5) && ((desk[row + 1][column] == 0))) {
				help = 1;
			}
			break;
		}
		return help;
	}

	/*
	 * vraci 1, pokud ma robot na prave strane hranu bludiste
	 */
	public int controlRightEdge() {
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

	/*
	 * vraci 1, pokud robot uz byl vepredu, vzadu a vlevo
	 */
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
						if ((column == 8)
								|| ((column < 8) && ((desk[row][column + 1] == 0) || (desk[row][column + 1] == 2)))) { // napravo
																														// od
																														// robota
							help = 1;
						}
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
						if ((row == 5)
								|| ((row < 5) && ((desk[row + 1][column] == 0) || (desk[row + 1][column] == 2)))) { // napravo
																													// od
																													// robota
							// System.out.println("presence 1");
							help = 1;
						}
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
						if ((column == 0)
								|| ((column > 0) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2)))) { // nalevo
																														// od
							// robota
							// System.out.println("coordinate " + row + " " + column + " " + direct);
							// System.out.println("presence 2");
							help = 1;
						}
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
						if ((row == 0)
								|| ((row > 0) && ((desk[row - 1][column] == 0) || (desk[row - 1][column] == 2)))) { // nalevo
							// od
// robota
							// System.out.println("presence 3");
							help = 1;
						}
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
		if ((sampleDistance[0] < 0.25) && (sampleDistance[0] > 0)) {
			switch (direct) {
			case 0:
				if ((column > 0) && (desk[row][column - 1] != 0)) {
					desk[row][column - 1] = 2;
				//	 System.out.println("measureL " + row + " " + (column - 1));
				}
				break;
			case 1:
				if ((row > 0) && (desk[row - 1][column] != 0)) {
					desk[row - 1][column] = 2;
				//	 System.out.println("measureL " + (row - 1) + " " + column);
				}
				break;
			case 2:
				if ((column < 8) && (desk[row][column + 1] != 0)) {
					desk[row][column + 1] = 2;
				//	 System.out.println("measureL " + row + " " + (column + 1));
				}
				break;
			case 3:
				if ((row < 5) && (desk[row + 1][column] != 0)) {
					desk[row + 1][column] = 2;
				//	 System.out.println("measureL " + (row + 1) + " " + column);
				}
				break;
			}
		} else if (sampleDistance[0] > 0.25) {
			switch (direct) {
			case 0:
				if ((column > 0) && (desk[row][column - 1] == 2)) {
					desk[row][column - 1] = 3;
				//	 System.out.println("restartL " + row + " " + (column - 1));
				}
				break;
			case 1:
				if ((row > 0) && (desk[row - 1][column] == 2)) {
					desk[row - 1][column] = 3;
				//	 System.out.println("restartL " + (row - 1) + " " + column);
				}
				break;
			case 2:
				if ((column < 8) && (desk[row][column + 1] == 2)) {
					desk[row][column + 1] = 3;
				//	 System.out.println("restartL " + row + " " + (column + 1));
				}
				break;
			case 3:
				if ((row < 5) && (desk[row + 1][column] == 2)) {
					desk[row + 1][column] = 3;
				//	 System.out.println("restartL " + (row + 1) + " " + column);
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
		if ((sampleDistance[0] < 0.25) && (sampleDistance[0] > 0)) {
			switch (direct) {
			case 0:
				if ((column < 8) && (desk[row][column + 1] != 0)) {
					desk[row][column + 1] = 2;
				//	 System.out.println("measureR " + row + " " + (column + 1));
				}
				break;
			case 1:
				if ((row < 5) && (desk[row + 1][column] != 0)) {
					desk[row + 1][column] = 2;
				//	 System.out.println("measureR " + (row + 1) + " " + column);
				}
				break;
			case 2:
				if ((column > 0) && (desk[row][column - 1] != 0)) {
					desk[row][column - 1] = 2;
				//	 System.out.println("measureR " + row + " " + (column - 1));
				}
				break;
			case 3:
				if ((row > 0) && (desk[row - 1][column] != 0)) {
					desk[row - 1][column] = 2;
				//	 System.out.println("measureR " + (row - 1) + " " + column);
				}
				break;
			}
		} else if (sampleDistance[0] > 0.25) {
			switch (direct) {
			case 0:
				if ((column < 8) && (desk[row][column + 1] == 2)) {
					desk[row][column + 1] = 3;
				//	 System.out.println("restartR " + row + " " + (column + 1));
				}
				break;
			case 1:
				if ((row < 5) && (desk[row + 1][column] == 2)) {
					desk[row + 1][column] = 3;
				//	 System.out.println("restartR " + (row + 1) + " " + column);
				}
				break;
			case 2:
				if ((column > 0) && (desk[row][column - 1] == 2)) {
					desk[row][column - 1] = 3;
				//	 System.out.println("restartR " + row + " " + (column - 1));
				}
				break;
			case 3:
				if ((row > 0) && (desk[row - 1][column] == 2)) {
					desk[row - 1][column] = 3;
				//	 System.out.println("restartR " + (row - 1) + " " + column);
				}
				break;
			}
		}
	}

	/*
	 * vrati robota o 1 policku vzad
	 */
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

	/*
	 * rotuje robota v systemu souradnic doleva
	 */
	public void rotationL() {
		if (direct == 0) {
			direct = 3;
		} else {
			direct--;
		}
		System.out.println("rot left " + row + " " + column + " " + direct);
	}

	/*
	 * rotuje robota v systemu souradnic doprava
	 */
	public void rotationR() {
		if (direct == 3) {
			direct = 0;
		} else {
			direct++;
		}
		System.out.println("rot right " + row + " " + column + " " + direct);
	}

	/*
	 * rotuje robota v systemu souradnic o 180 stupnu
	 */
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

	/*
	 * spusti prohledavani do sirky
	 */
	public int findWay() {
		BFS.inicialize(desk, row, column, direct);
		way = BFS.getWay(); // vrati cestu od zacatku do konce (bez aktu�ln�ho pole)
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

	/*
	 * resetuje BFS
	 */
	public void eraseWay() {
		way.clear();
		BFS.stop();
	}

	/*
	 * vrati robota v souradnem systemu na vychozi pozici
	 */
	public void reset() {
		row = 3;
		column = 4;
		direct = 0;
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