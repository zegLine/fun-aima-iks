package eugene.osm.agent.online;

import aima.core.agent.Agent;
import aima.core.agent.impl.DynamicPercept;
import aima.core.environment.map.BidirectionalMapProblem;
import aima.core.environment.map.MapFunctions;
import aima.core.environment.map.MoveToAction;
import aima.core.search.framework.problem.GeneralProblem;
import aima.core.search.framework.problem.OnlineSearchProblem;
import aima.core.search.framework.problem.Problem;
import aimax.osm.gui.fx.applications.OnlineAgentOsmApp;

import java.util.List;
import java.util.function.ToDoubleFunction;

public class ExtendedOnlineAgentOsmApp extends OnlineAgentOsmApp {

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public String getTitle() {
        return "Eugene's Online Agent OSM App";
    }

    @Override
    protected Agent<DynamicPercept, MoveToAction> createAgent(List<String> locations) {
        String goal = locations.get(1);
        Problem<String, MoveToAction> p = new BidirectionalMapProblem(map, null, goal);
        OnlineSearchProblem<String, MoveToAction> osp = new GeneralProblem<>
                (null, p::getActions, null, p::testGoal, p::getStepCosts);
        ToDoubleFunction<String> heuristic = state -> MapFunctions.getSLD(state, goal, map);

        return new ImprovedLRTAStarAgent<>(osp, MapFunctions.createPerceptToStateFunction(), heuristic);
    }

}
