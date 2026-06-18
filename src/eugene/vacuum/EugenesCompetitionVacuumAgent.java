package eugene.vacuum;

import aima.core.agent.Action;
import aima.core.agent.impl.SimpleAgent;
import aima.core.environment.vacuum.VacuumPercept;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashSet;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;

import static aima.core.environment.vacuum.MazeVacuumEnvironment.*;

/**
 *
 */
public class EugenesCompetitionVacuumAgent extends SimpleAgent<VacuumPercept, Action> {

    // my position, the start square is the origin
    int x = 0;
    int y = 0;
    private final Set<String> set_visited = new HashSet<>();
    private final Deque<Action> bt = new ArrayDeque<>(); // steps that lead back home

    @Override
    public Optional<Action> act(VacuumPercept percept) {
        set_visited.add(key(x, y));

        // clean first
        if (percept.getCurrState() == LocationState.Dirty)
            return Optional.of(ACTION_SUCK);

        // go to a neighbour that is notseen yet
        Action move = firstUnvisited(percept);
        if (move != null) {
            bt.push(opposite(move));
            apply(move);
            return Optional.of(move);
        }

        // dead end -> retrace one step toward the start
        if (!bt.isEmpty()) {
            Action back = bt.pop();
            apply(back);
            return Optional.of(back);
        }

        // everything that we can do is done
        return Optional.of(ACTION_SUCK);
    }

    private Action firstUnvisited(VacuumPercept p) {
        if (canGo(p, ATT_CAN_MOVE_UP) && unseen(x, y + 1)) return ACTION_MOVE_UP;
        if (canGo(p, ATT_CAN_MOVE_RIGHT) && unseen(x + 1, y)) return ACTION_MOVE_RIGHT;
        if (canGo(p, ATT_CAN_MOVE_DOWN) && unseen(x, y - 1)) return ACTION_MOVE_DOWN;
        if (canGo(p, ATT_CAN_MOVE_LEFT) && unseen(x - 1, y)) return ACTION_MOVE_LEFT;
        return null;
    }

    private boolean canGo(VacuumPercept p, String att) {
        return Objects.equals(p.getAttribute(att), true);
    }

    private boolean unseen(int nx, int ny) {
        return !set_visited.contains(key(nx, ny));
    }

    private void apply(Action move) {
        if (move == ACTION_MOVE_UP) y++;
        else if (move == ACTION_MOVE_DOWN) y--;
        else if (move == ACTION_MOVE_LEFT) x--;
        else if (move == ACTION_MOVE_RIGHT) x++;
    }

    private Action opposite(Action move) {
        if (move == ACTION_MOVE_UP) return ACTION_MOVE_DOWN;
        if (move == ACTION_MOVE_DOWN) return ACTION_MOVE_UP;
        if (move == ACTION_MOVE_LEFT) return ACTION_MOVE_RIGHT;
        return ACTION_MOVE_LEFT;
    }

    private String key(int kx, int ky) {
        return kx + "," + ky;
    }
}
