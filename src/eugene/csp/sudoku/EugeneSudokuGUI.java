package eugene.csp.sudoku;

import aima.core.search.csp.Assignment;
import aima.core.search.csp.Variable;
import aima.core.search.csp.solver.BackjumpingBacktrackingSolver;
import aima.core.search.csp.solver.CspHeuristics;
import aima.core.search.csp.solver.CspListener;
import aima.core.search.csp.solver.CspSolver;
import aima.core.search.csp.solver.FlexibleBacktrackingSolver;
import aima.core.search.csp.solver.MinConflictsSolver;
import aima.core.search.csp.solver.inference.AC3Strategy;
import aima.core.search.csp.solver.inference.ForwardCheckingStrategy;
import aima.gui.fx.framework.IntegrableApplication;
import aima.gui.fx.framework.Parameter;
import aima.gui.fx.framework.TaskExecutionPaneBuilder;
import aima.gui.fx.framework.TaskExecutionPaneCtrl;
import aima.gui.fx.views.SudokuViewCtrl;
import javafx.application.Platform;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.layout.StackPane;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * Sudoku CSP application. Adapted from NQueensCspApp to demonstrate how
 * different CSP solution strategies solve a 9x9 Sudoku puzzle.
 */
public class EugeneSudokuGUI extends IntegrableApplication {

    /** Example taken from https://en.wikipedia.org/wiki/Sudoku. */
    public final static String puzzle1 = "" +
            "53..7...." +
            "6..195..." +
            ".98....6." +
            "8...6...3" +
            "4..8.3..1" +
            "7...2...6" +
            ".6....28." +
            "...419..5" +
            "....8..79";

    /** Example taken from https://de.wikipedia.org/wiki/Sudoku. */
    public final static String puzzle2 = "" +
            ".3......." +
            "...195..." +
            "..8....6." +
            "8...6...." +
            "4..8....1" +
            "....2...." +
            ".6....28." +
            "...419..5" +
            ".......7.";

    /** Example taken from http://sudoku.tagesspiegel.de/sudoku-sehr-schwer. */
    public final static String puzzle3 = "" +
            ".....9.7." +
            "....82.5." +
            "327....4." +
            ".16.4...." +
            ".5....3.." +
            "....9.7.." +
            "...6....5" +
            "8.2......" +
            "..42....8";

    public static void main(String[] args) {
        launch(args);
    }

    private final static int SIZE = 9;

    private final static String PARAM_PUZZLE = "puzzle";
    private final static String PARAM_STRATEGY = "strategy";
    private final static String PARAM_VAR_SELECT = "varSelect";
    private final static String PARAM_VAL_SELECT = "valOrder";
    private final static String PARAM_INFERENCE = "inference";

    private SudokuViewCtrl stateViewCtrl;
    private TaskExecutionPaneCtrl taskPaneCtrl;
    private SudokuCSP csp;
    private CspSolver<Variable, Integer> solver;
    private final CspListener.StepCounter<Variable, Integer> stepCounter = new CspListener.StepCounter<>();
    private final boolean[][] fixed = new boolean[SIZE][SIZE];

    @Override
    public String getTitle() {
        return "Sudoku CSP App";
    }

    @Override
    public Pane createRootPane() {
        BorderPane root = new BorderPane();

        StackPane stateView = new StackPane();
        stateViewCtrl = new SudokuViewCtrl(stateView);

        List<Parameter> params = createParameters();

        TaskExecutionPaneBuilder builder = new TaskExecutionPaneBuilder();
        builder.defineParameters(params);
        builder.defineStateView(stateView);
        builder.defineInitMethod(this::initialize);
        builder.defineTaskMethod(this::startExperiment);
        taskPaneCtrl = builder.getResultFor(root);
        taskPaneCtrl.setParam(TaskExecutionPaneCtrl.PARAM_EXEC_SPEED, 0);

        return root;
    }

    protected List<Parameter> createParameters() {
        Parameter p0 = new Parameter(PARAM_PUZZLE, "Puzzle 1", "Puzzle 2", "Puzzle 3");
        Parameter p1 = new Parameter(PARAM_STRATEGY, "Backtracking", "Backjumping", "Min-Conflicts");
        Parameter p2 = new Parameter(PARAM_VAR_SELECT, "Default", "MRV", "DEG", "MRV&DEG");
        Parameter p3 = new Parameter(PARAM_VAL_SELECT, "Default", "LCV");
        Parameter p4 = new Parameter(PARAM_INFERENCE, "None", "Forward Checking", "AC3");
        p2.setDependency(PARAM_STRATEGY, "Backtracking", "Backjumping");
        p3.setDependency(PARAM_STRATEGY, "Backtracking", "Backjumping");
        p4.setDependency(PARAM_STRATEGY, "Backtracking");
        return Arrays.asList(p0, p1, p2, p3, p4);
    }

    @Override
    public void initialize() {
        csp = new SudokuCSP(SIZE);
        applyPuzzle(selectedPuzzle());

        Object strategy = taskPaneCtrl.getParamValue(PARAM_STRATEGY);
        if (strategy.equals("Backtracking")) {
            FlexibleBacktrackingSolver<Variable, Integer> bSolver = new FlexibleBacktrackingSolver<>();
            switch ((String) taskPaneCtrl.getParamValue(PARAM_VAR_SELECT)) {
                case "MRV": bSolver.set(CspHeuristics.mrv()); break;
                case "DEG": bSolver.set(CspHeuristics.deg()); break;
                case "MRV&DEG": bSolver.set(CspHeuristics.mrvDeg()); break;
            }
            switch ((String) taskPaneCtrl.getParamValue(PARAM_VAL_SELECT)) {
                case "LCV": bSolver.set(CspHeuristics.lcv()); break;
            }
            switch ((String) taskPaneCtrl.getParamValue(PARAM_INFERENCE)) {
                case "Forward Checking": bSolver.set(new ForwardCheckingStrategy<>()); break;
                case "AC3": bSolver.set(new AC3Strategy<>()); break;
            }
            solver = bSolver;
        } else if (strategy.equals("Backjumping")) {
            BackjumpingBacktrackingSolver<Variable, Integer> bSolver = new BackjumpingBacktrackingSolver<>();
            switch ((String) taskPaneCtrl.getParamValue(PARAM_VAR_SELECT)) {
                case "MRV": bSolver.set(CspHeuristics.mrv()); break;
                case "DEG": bSolver.set(CspHeuristics.deg()); break;
                case "MRV&DEG": bSolver.set(CspHeuristics.mrvDeg()); break;
            }
            switch ((String) taskPaneCtrl.getParamValue(PARAM_VAL_SELECT)) {
                case "LCV": bSolver.set(CspHeuristics.lcv()); break;
            }
            solver = bSolver;
        } else if (strategy.equals("Min-Conflicts")) {
            solver = new MinConflictsSolver<>(1000);
        }
        solver.addCspListener(stepCounter);
        solver.addCspListener((c, assign, var) -> { if (assign != null) updateStateView(assign); });
        stepCounter.reset();
        taskPaneCtrl.setStatus("");
    }

    @Override
    public void cleanup() {
        taskPaneCtrl.cancelExecution();
    }

    public void startExperiment() {
        Optional<Assignment<Variable, Integer>> solution = solver.solve(csp);
        if (solution.isPresent())
            Platform.runLater(() -> renderAssignment(solution.get()));
    }

    private String selectedPuzzle() {
        switch ((String) taskPaneCtrl.getParamValue(PARAM_PUZZLE)) {
            case "Puzzle 2": return puzzle2;
            case "Puzzle 3": return puzzle3;
            default: return puzzle1;
        }
    }

    private void applyPuzzle(String puzzle) {
        stateViewCtrl.clear(true);
        for (int r = 0; r < SIZE; r++)
            for (int c = 0; c < SIZE; c++)
                fixed[r][c] = false;
        for (int i = 0; i < puzzle.length(); i++) {
            char ch = puzzle.charAt(i);
            if (ch >= '1' && ch <= '9') {
                int col = i % SIZE + 1;
                int row = i / SIZE + 1;
                int digit = ch - '0';
                stateViewCtrl.fixDigit(col, row, digit);
                csp.setValue(row, col, digit);
                fixed[row - 1][col - 1] = true;
            }
        }
    }

    /** Variable names are of the form "V{row},{col}" (1-based). */
    private int[] parseRowCol(Variable var) {
        String s = var.getName().substring(1);
        int comma = s.indexOf(',');
        int row = Integer.parseInt(s.substring(0, comma));
        int col = Integer.parseInt(s.substring(comma + 1));
        return new int[] { row, col };
    }

    private void renderAssignment(Assignment<Variable, Integer> assignment) {
        for (Variable var : assignment.getVariables()) {
            int[] rc = parseRowCol(var);
            if (!fixed[rc[0] - 1][rc[1] - 1])
                stateViewCtrl.setDigit(rc[1], rc[0], assignment.getValue(var));
        }
    }

    /**
     * Caution: While the background thread should be slowed down, updates of
     * the GUI have to be done in the GUI thread!
     */
    private void updateStateView(Assignment<Variable, Integer> assignment) {
        Platform.runLater(() -> {
            renderAssignment(assignment);
            taskPaneCtrl.setStatus(stepCounter.getResults().toString());
        });
        taskPaneCtrl.waitAfterStep();
    }
}
