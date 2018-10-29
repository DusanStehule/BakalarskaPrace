import java.util.HashMap;
import java.util.LinkedList;

public class BFS {
	
	int[][] desk = new int [6][9];
	int row;
	int column;
	int direct;
	HashMap<Integer, Integer> parent = new HashMap<Integer, Integer>();
	LinkedList<Integer> open = new LinkedList<Integer>();
	LinkedList<Integer> close = new LinkedList<Integer>();
	LinkedList<Integer> expanded = new LinkedList<Integer>();
	LinkedList<Integer> way = new LinkedList<Integer>();

	public BFS() {
	}
	
	public void inicialize(int[][] desk, int row, int column, int direct) {
		this.desk = desk;
		this.row = row;
		this.column = column;
		this.direct = direct;
		findWay();
	}
	
	public void stop() {
		open.clear();
		close.clear();
		expanded.clear();
		way.clear();
	}

	private void findWay() {
		int field;
		int goal = 0;
		open.addAll(expand(hashIn(row, column)));
		for (int i = 0; i < open.size(); i++) {
			field = open.get(i);
			parent.put(field, hashIn(row, column));
		}
		
		while (!open.isEmpty()) {
			field = open.get(0);
			close.add(field);
			open.remove((Object)field);
			if (isTarget(field)) {
				goal = field;
				break;
			} else {
				expanded.addAll(expand(field));
				for (int i = 0; i < expanded.size(); i++) {
					if ((!open.contains(expanded.get(i))) && (!close.contains(expanded.get(i)))) {
						open.add(expanded.get(i));
						parent.put(expanded.get(i), field);
					}
				}
				expanded.clear();
			}
		}
		System.out.println("goal " + (goal-10));
		open.clear();
		close.clear();
		expanded.clear();
		
		while (goal != (hashIn(row, column))) {
			way.addFirst(goal);
			goal = parent.get(goal);
		}
	}
	
	private LinkedList<Integer> expand(int hash) {
		LinkedList<Integer> exp = new LinkedList<Integer>();
		int row = (hash / 10) - 1;
		int column = hash % 10;
		int help;
		
		if ((row < 5) && (desk[row + 1][column] != 2) && (desk[row + 1][column] != 4)) {
			help = (row + 2) * 10 + column;
			exp.add(help);
		}
		
		if ((row > 0) && (desk[row - 1][column] != 2) && (desk[row - 1][column] != 4)) {
			help = (row) * 10 + column;
			exp.add(help);
		}
		
		if ((column < 8) && (desk[row][column + 1] != 2) && (desk[row][column + 1] != 4)) {
			help = (row + 1) * 10 + column + 1;
			exp.add(help);
		}
		
		if ((column > 0) && (desk[row][column - 1] != 2) && (desk[row][column - 1] != 4)) {
			help = (row + 1) * 10 + column - 1;
			exp.add(help);
		}
		
		return exp;
	}
	
	private int hashIn(int row, int column) {
		return (row + 1) * 10 + column;
	}
	
	private boolean isTarget(int field) {
		boolean ret = false;
		if (desk[(field / 10) - 1][field % 10] == 3) {
			ret = true;
		}
		return ret;
	}
	
	public LinkedList<Integer> getWay() {
		return way;
	}
	
}
