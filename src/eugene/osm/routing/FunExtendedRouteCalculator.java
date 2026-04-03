package eugene.osm.routing;

import aima.core.search.framework.Node;
import aima.core.search.framework.problem.Problem;
import aima.core.search.framework.problem.StepCostFunction;
import aimax.osm.data.MapWayFilter;
import aimax.osm.data.OsmMap;
import aimax.osm.data.Position;
import aimax.osm.data.entities.MapNode;
import aimax.osm.routing.OsmMoveAction;
import aimax.osm.routing.OsmSldHeuristicFunction;
import aimax.osm.routing.RouteCalculator;
import aimax.osm.routing.RouteFindingProblem;

import java.util.function.ToDoubleFunction;

public class FunExtendedRouteCalculator extends RouteCalculator {

    static class TimeStepCostFunction implements StepCostFunction<MapNode, OsmMoveAction> {

        @Override
        public double applyAsDouble(MapNode mapNode, OsmMoveAction osmMoveAction, MapNode sDelta) {
            return osmMoveAction.getTravelDistance(); // so far, will optimize later for car and bike
        }
    }

    static class TimeHeuristicFunction implements ToDoubleFunction<Node<MapNode, OsmMoveAction>> {
        private final MapNode goalState;

        public TimeHeuristicFunction(MapNode goalState) {
            this.goalState = goalState;
        }

        @Override
        public double applyAsDouble(Node<MapNode, OsmMoveAction> node) {
            return (new Position(node.getState())).getDistKM(goalState); // will also change
        }
    }

    @Override
    protected Problem<MapNode, OsmMoveAction> createProblem(MapNode[] pNodes, OsmMap map,
                                                            MapWayFilter wayFilter, boolean ignoreOneways, int taskSelection) {
        return new RouteFindingProblem(pNodes[0], pNodes[1], wayFilter,
                ignoreOneways, new TimeStepCostFunction());
    }

    @Override
    protected ToDoubleFunction<Node<MapNode, OsmMoveAction>> createHeuristicFunction(MapNode[] pNodes,
                                                                                     int taskSelection) {
        return new TimeHeuristicFunction(pNodes[1]);
    }
}
