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
		desk[3][3] = 2; // start field
		desk[3][4] = 2; // start field
		desk[3][5] = 2; // start field
	}

	/*
	 * robot move one field forward in the coordinate system
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
	 * 1 if robot was there
	 */
	public int controlForward() {
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
	 * 1 if edge of labyrinth is before robot
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
	 * 1 if robot was there or barrier is there
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
	 * 1 if robot was on the field right
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
	 * 1 if robot was on the field left
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
	 * 1 if edge of labyrinth is on the robot's right
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
	 * 1 if robot was on the field forward, back, left and right or is barrier
	 * forward, back, left and right
	 */
	public int controlPresenceAround() {
		int help = 0;
		switch (direct) {
		case 0:
			if ((row == 0) || ((row > 0) && ((desk[row - 1][column] == 0) || (desk[row - 1][column] == 2)))) {
				if ((row == 5) || ((row < 5) && ((desk[row + 1][column] == 0) || (desk[row + 1][column] == 2)))) {
					if ((column == 0)
							|| ((column > 0) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2)))) {
						if ((column == 8)
								|| ((column < 8) && ((desk[row][column + 1] == 0) || (desk[row][column + 1] == 2)))) {
							help = 1;
						}
					}
				}
			}
			break;
		case 1:
			if ((column == 8) || ((column < 8) && ((desk[row][column + 1] == 0) || (desk[row][column + 1] == 2)))) {
				if ((column == 0) || ((column > 0) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2)))) {
					if ((row == 0) || ((row > 0) && ((desk[row - 1][column] == 0) || (desk[row - 1][column] == 2)))) {
						if ((row == 5)
								|| ((row < 5) && ((desk[row + 1][column] == 0) || (desk[row + 1][column] == 2)))) {
							help = 1;
						}
					}
				}
			}
			break;
		case 2:
			if ((row == 5) || ((row < 5) && ((desk[row + 1][column] == 0) || (desk[row + 1][column] == 2)))) {
				if ((row == 0) || ((row > 0) && ((desk[row - 1][column] == 0) || (desk[row - 1][column] == 2)))) {
					if ((column == 8)
							|| ((column < 8) && ((desk[row][column + 1] == 0) || (desk[row][column + 1] == 2)))) {
						if ((column == 0)
								|| ((column > 0) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2)))) {
							help = 1;
						}
					}
				}
			}
			break;
		case 3:
			if ((column == 0) || ((column > 0) && ((desk[row][column - 1] == 0) || (desk[row][column - 1] == 2)))) {
				if ((column == 8) || ((column < 8) && ((desk[row][column + 1] == 0) || (desk[row][column + 1] == 2)))) {
					if ((row == 5) || ((row < 5) && ((desk[row + 1][column] == 0) || (desk[row + 1][column] == 2)))) {
						if ((row == 0)
								|| ((row > 0) && ((desk[row - 1][column] == 0) || (desk[row - 1][column] == 2)))) {
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
	 * maps barriers before robot after touch with touch sensor
	 */
	public void measureForward() {
		switch (direct) {
		case 0:
			if ((row > 0) && (desk[row - 1][column] != 0)) {
				desk[row - 1][column] = 2;
			}
			break;
		case 1:
			if ((column < 8) && (desk[row][column + 1] != 0)) {
				desk[row][column + 1] = 2;
			}
			break;
		case 2:
			if ((row < 5) && (desk[row + 1][column] != 0)) {
				desk[row + 1][column] = 2;
			}
			break;
		case 3:
			if ((column > 0) && (desk[row][column - 1] != 0)) {
				desk[row][column - 1] = 2;
			}
			break;
		}
	}

	/*
	 * maps barriers on the robot's left
	 */
	public void measureLeft() {
		distanceSampler.fetchSample(sampleDistance, 0);
		if ((sampleDistance[0] < 0.25) && (sampleDistance[0] > 0)) {
			switch (direct) {
			case 0:
				if ((column > 0) && (desk[row][column - 1] != 0)) {
					desk[row][column - 1] = 2;
				}
				break;
			case 1:
				if ((row > 0) && (desk[row - 1][column] != 0)) {
					desk[row - 1][column] = 2;
				}
				break;
			case 2:
				if ((column < 8) && (desk[row][column + 1] != 0)) {
					desk[row][column + 1] = 2;
				}
				break;
			case 3:
				if ((row < 5) && (desk[row + 1][column] != 0)) {
					desk[row + 1][column] = 2;
				}
				break;
			}
		} else if (sampleDistance[0] > 0.25) {
			switch (direct) {
			case 0:
				if ((column > 0) && (desk[row][column - 1] == 2)) {
					desk[row][column - 1] = 3;
				}
				break;
			case 1:
				if ((row > 0) && (desk[row - 1][column] == 2)) {
					desk[row - 1][column] = 3;
				}
				break;
			case 2:
				if ((column < 8) && (desk[row][column + 1] == 2)) {
					desk[row][column + 1] = 3;
				}
				break;
			case 3:
				if ((row < 5) && (desk[row + 1][column] == 2)) {
					desk[row + 1][column] = 3;
				}
				break;
			}
		}
	}

	/*
	 * maps barriers on the robot's right
	 */
	public void measureRight() {
		distanceSampler.fetchSample(sampleDistance, 0);
		if ((sampleDistance[0] < 0.25) && (sampleDistance[0] > 0)) {
			switch (direct) {
			case 0:
				if ((column < 8) && (desk[row][column + 1] != 0)) {
					desk[row][column + 1] = 2;
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
		} else if (sampleDistance[0] > 0.25) {
			switch (direct) {
			case 0:
				if ((column < 8) && (desk[row][column + 1] == 2)) {
					desk[row][column + 1] = 3;
				}
				break;
			case 1:
				if ((row < 5) && (desk[row + 1][column] == 2)) {
					desk[row + 1][column] = 3;
				}
				break;
			case 2:
				if ((column > 0) && (desk[row][column - 1] == 2)) {
					desk[row][column - 1] = 3;
				}
				break;
			case 3:
				if ((row > 0) && (desk[row - 1][column] == 2)) {
					desk[row - 1][column] = 3;
				}
				break;
			}
		}
	}

	/*
	 * returns the robot one field back 
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
	 * rotate robot to the left in the coordinate system
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
	 * rotate robot to the right in the coordinate system
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
	 * rotate robot to the back in the coordinate system
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
	 * starts BFS
	 */
	public int findWay() {
		BFS.inicialize(desk, row, column, direct);
		way = BFS.getWay(); // vrati cestu od zacatku do konce (bez aktuálního pole)
		return way.size();
	}

	/*
	 * this function returns direct, which is to unexplored field
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
	 * this function returns direct, which is to unexplored field and erase first field
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
	 * reset BFS
	 */
	public void eraseWay() {
		way.clear();
		BFS.stop();
	}

	/*
	 * returns robot on the start in the coordinate system
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
