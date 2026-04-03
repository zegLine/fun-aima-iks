package eugene.vacuum;

import aima.core.agent.Action;
import aima.core.agent.Model;
import aima.core.agent.impl.DynamicState;
import aima.core.agent.impl.SimpleAgent;
import aima.core.agent.impl.aprog.ModelBasedReflexAgentProgram;
import aima.core.agent.impl.aprog.simplerule.ANDCondition;
import aima.core.agent.impl.aprog.simplerule.EQUALCondition;
import aima.core.agent.impl.aprog.simplerule.NOTCondition;
import aima.core.agent.impl.aprog.simplerule.Rule;
import aima.core.environment.vacuum.VacuumPercept;

import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Set;

import static aima.core.environment.vacuum.MazeVacuumEnvironment.ATT_CAN_MOVE_LEFT;
import static aima.core.environment.vacuum.MazeVacuumEnvironment.ATT_CAN_MOVE_RIGHT;
import static aima.core.environment.vacuum.VacuumEnvironment.*;

/**
 * Model-based reflex vacuum agent with internal state.
 * Tracks visited locations and uses a sweep direction strategy:
 * moves in one direction until hitting a wall, then reverses.
 * This avoids oscillation and ensures systematic coverage.
 *
 * @author Eugen Z.
 */
public class EugenModelBasedReflexVacuumAgent extends SimpleAgent<VacuumPercept, Action> {

	private static final String ATT_CURRENT_STATE = "currentState";
	private static final String ATT_MOVING_RIGHT = "movingRight";
	private static final String ATT_VISITED_LOCATIONS = "visitedLocations";


	public EugenModelBasedReflexVacuumAgent() {
		super(new ModelBasedReflexAgentProgram<VacuumPercept, Action>() {
			@Override
			protected void init() {
				DynamicState initialState = new DynamicState();
				// Start by moving right as the default sweep direction
				initialState.setAttribute(ATT_MOVING_RIGHT, true);
				initialState.setAttribute(ATT_VISITED_LOCATIONS, new HashSet<String>());
				setState(initialState);
				setRules(getRuleSet());
			}

			@SuppressWarnings("unchecked")
			@Override
			protected DynamicState updateState(DynamicState state, Action anAction, VacuumPercept percept,
											   Model model) {
				Object currState = percept.getCurrState();
				Object canMoveLeft = percept.getAttribute(ATT_CAN_MOVE_LEFT);
				Object canMoveRight = percept.getAttribute(ATT_CAN_MOVE_RIGHT);

				state.setAttribute(ATT_CURRENT_STATE, currState);
				state.setAttribute(ATT_CAN_MOVE_LEFT, canMoveLeft);
				state.setAttribute(ATT_CAN_MOVE_RIGHT, canMoveRight);

				// Track visited locations in internal state
				String currentLocation = percept.getCurrLocation();
				Set<String> visited = (Set<String>) state.getAttribute(ATT_VISITED_LOCATIONS);
				if (visited == null) {
					visited = new HashSet<>();
				}
				visited.add(currentLocation);
				state.setAttribute(ATT_VISITED_LOCATIONS, visited);

				// Sweep direction logic: reverse when hitting a wall
				boolean movingRight = Boolean.TRUE.equals(state.getAttribute(ATT_MOVING_RIGHT));

				if (movingRight && !Boolean.TRUE.equals(canMoveRight)) {
					// Hit right wall -> reverse to move left
					state.setAttribute(ATT_MOVING_RIGHT, false);
				} else if (!movingRight && !Boolean.TRUE.equals(canMoveLeft)) {
					// Hit left wall -> reverse to move right
					state.setAttribute(ATT_MOVING_RIGHT, true);
				}

				return state;
			}
		});
	}

	private static Set<Rule<Action>> getRuleSet() {
		// Note: LinkedHashSet preserves iteration order (i.e. implied precedence).
		Set<Rule<Action>> rules = new LinkedHashSet<>();

		// Rule 1: If current location is dirty, suck (highest priority)
		rules.add(new Rule<>(new EQUALCondition(ATT_CURRENT_STATE, LocationState.Dirty), ACTION_SUCK));

		// Rule 2: If moving right and can move right, move right
		rules.add(new Rule<>(
				new ANDCondition(
						new EQUALCondition(ATT_MOVING_RIGHT, true),
						new EQUALCondition(ATT_CAN_MOVE_RIGHT, true)),
				ACTION_MOVE_RIGHT));

		// Rule 3: If moving left and can move left, move left
		rules.add(new Rule<>(
				new ANDCondition(
						new NOTCondition(new EQUALCondition(ATT_MOVING_RIGHT, true)),
						new EQUALCondition(ATT_CAN_MOVE_LEFT, true)),
				ACTION_MOVE_LEFT));

		// Rule 4: Fallback - if can move right, move right (after direction was just reversed)
		rules.add(new Rule<>(new EQUALCondition(ATT_CAN_MOVE_RIGHT, true), ACTION_MOVE_RIGHT));

		// Rule 5: Fallback - if can move left, move left
		rules.add(new Rule<>(new EQUALCondition(ATT_CAN_MOVE_LEFT, true), ACTION_MOVE_LEFT));

		return rules;
	}
}
