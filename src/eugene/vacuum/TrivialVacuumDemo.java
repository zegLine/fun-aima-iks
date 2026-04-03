package eugene.vacuum;

import aima.core.agent.Action;
import aima.core.agent.Agent;
import aima.core.agent.Environment;
import aima.core.agent.EnvironmentListener;
import aima.core.agent.impl.SimpleEnvironmentView;
import aima.core.environment.vacuum.MazeVacuumEnvironment;
import aima.core.environment.vacuum.ModelBasedReflexVacuumAgent;
import aima.core.environment.vacuum.VacuumEnvironment;
import aima.core.environment.vacuum.VacuumPercept;

/**
 *
 * @author Eugen Z.
 */
public class TrivialVacuumDemo {

    private static double averagePerf = 0.0f;

    public static void main(String[] args) {
        int noRuns = 200;
        for (int i = 0; i < noRuns; i++) {
            double currentRunPerf = runAndGetPerformance();
            averagePerf += currentRunPerf;
        }
        averagePerf /= noRuns;
        System.out.println("average perf: " + averagePerf);
	}

    private static double runAndGetPerformance(){
        // create environment with random state of cleaning.
        Environment<VacuumPercept, Action> env = new MazeVacuumEnvironment(8, 1, 0.5, 0);
        EnvironmentListener<Object, Object> view = new SimpleEnvironmentView();
        env.addEnvironmentListener(view);

        Agent<VacuumPercept, Action> agent;
        agent = new EugenModelBasedReflexVacuumAgent();
        // agent = new ReflexVacuumAgent();
        // agent = new SimpleReflexVacuumAgent();
        // agent = new TableDrivenVacuumAgent();

        env.addAgent(agent);
        env.step(16);
        return env.getPerformanceMeasure(agent);
    }
}
