package eugene.csp.sudoku;

import aima.core.search.csp.CSP;
import aima.core.search.csp.Domain;
import aima.core.search.csp.Variable;
import aima.core.search.csp.examples.NotEqualConstraint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class SudokuCSP extends CSP<Variable, Integer> {

	private final int size;

	public SudokuCSP(int size) {
		this.size = size;
		int boxSize = (int) Math.sqrt(size);

		for (int r = 0; r < size; r++)
			for (int c = 0; c < size; c++)
				addVariable(new Variable("V" + (r+1) + "," + (c+1)));

		List<Integer> values = new ArrayList<>();
		for (int val = 1; val <= size; val++)
			values.add(val);
		Domain<Integer> digits = new Domain<>(values);

		for (Variable var : getVariables())
			setDomain(var, digits);

		for (int r = 0; r < size; r++) {
			for (int c = 0; c < size; c++) {
				Variable var1 = getVariables().get(r*size + c);

				for (int cc = c+1; cc < size; cc++) {
					Variable var2 = getVariables().get(r*size + cc);
					addConstraint(new NotEqualConstraint<>(var1, var2));
				}

				for (int rr = r+1; rr < size; rr++) {
					Variable var2 = getVariables().get(rr*size + c);
					addConstraint(new NotEqualConstraint<>(var1, var2));
				}

				int boxR = r / boxSize;
				int boxC = c / boxSize;
				for (int rr = r+1; rr < (boxR+1)*boxSize; rr++) {
					for (int cc = boxC*boxSize; cc < (boxC+1)*boxSize; cc++) {
						if (cc != c) {
							Variable var2 = getVariables().get(rr*size + cc);
							addConstraint(new NotEqualConstraint<>(var1, var2));
						}
					}
				}
			}
		}
	}

	public void setValue(int row, int col, int value) {
		if (row < 1 || row > size || col < 1 || col > size)
			throw new IllegalArgumentException("Position (" + row + "," + col + ") out of range 1.." + size);
		if (value < 1 || value > size)
			throw new IllegalArgumentException("Value " + value + " out of range 1.." + size);
		Variable var = getVariables().get((row - 1) * size + (col - 1));
		setDomain(var, new Domain<>(Collections.singletonList(value)));
	}

	public void setValues(int[][] grid) {
		for (int r = 0; r < grid.length; r++)
			for (int c = 0; c < grid[r].length; c++)
				if (grid[r][c] != 0)
					setValue(r + 1, c + 1, grid[r][c]);
	}
}
