package eugene.osm.routing;

import aima.core.search.framework.Node;
import aima.core.search.framework.problem.Problem;
import aima.core.search.framework.problem.StepCostFunction;
import aimax.osm.data.MapWayAttFilter;
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

    // --- Task selection indices ---
    // 0 = Distance, 1 = Distance (Car), 2 = Distance (Bike), 3 = Time (Car), 4 = Fun (Cyclist)
    @Override
    public String[] getTaskSelectionOptions() {
        return new String[] { "Distance", "Distance (Car)", "Distance (Bike)", "Time (Car)", "Fun (Cyclist)" };
    }

    @Override
    protected MapWayFilter createMapWayFilter(OsmMap map, int taskSelection) {
        if (taskSelection == 3) // Time (Car)
            return MapWayAttFilter.createCarWayFilter();
        else if (taskSelection == 4) // Fun (Cyclist)
            return MapWayAttFilter.createBicycleWayFilter();
        else
            return super.createMapWayFilter(map, taskSelection);
    }

    // Time (Car)

    /** Returns a plausible average speed in km/h for a given highway type. */
    private static double getCarSpeedKmH(String highwayType) {
        if (highwayType == null) return 50.0;
        switch (highwayType) {
            case "motorway":      return 120.0;
            case "trunk":         return 100.0;
            case "primary":       return 80.0;
            case "secondary":     return 60.0;
            case "tertiary":      return 50.0;
            case "residential":   return 30.0;
            case "service":       return 20.0;
            case "unclassified":  return 40.0;
            default:              return 50.0;
        }
    }

    static final double MAX_CAR_SPEED_KMH = 120.0; // freeway — used for admissible heuristic

    static class TimeStepCostFunction implements StepCostFunction<MapNode, OsmMoveAction> {
        @Override
        public double applyAsDouble(MapNode from, OsmMoveAction action, MapNode to) {
            double distKm = action.getTravelDistance();
            String highwayType = action.getWay().getAttributeValue("highway");
            double speedKmH = getCarSpeedKmH(highwayType);
            return distKm / speedKmH; // cost in hours
        }
    }

    static class TimeHeuristicFunction implements ToDoubleFunction<Node<MapNode, OsmMoveAction>> {
        private final MapNode goalState;

        public TimeHeuristicFunction(MapNode goalState) {
            this.goalState = goalState;
        }

        @Override
        public double applyAsDouble(Node<MapNode, OsmMoveAction> node) {
            double sldKm = new Position(node.getState()).getDistKM(goalState);
            return sldKm / MAX_CAR_SPEED_KMH; // admissible: assumes max speed
        }
    }

    // Fun (Cyclist)

    /**
     * Returns a cost factor per km for cycling "fun". Lower factor is more fun.
     * Cycleways and paths are cheap, busy roads are not fu.
     */
    private static double getFunCostFactor(String highwayType) {
        if (highwayType == null) return 5.0;
        switch (highwayType) {
            case "cycleway":      return 1.0;  // most fun
            case "path":          return 1.5;
            case "track":         return 2.0;
            case "footway":       return 2.5;
            case "residential":   return 3.0;
            case "service":       return 3.5;
            case "tertiary":      return 4.0;
            case "secondary":     return 6.0;
            case "primary":       return 8.0;
            default:              return 5.0;
        }
    }

    static final double MIN_FUN_COST_FACTOR = 1.0; // cycleway — used for admissible heuristic

    static class FunStepCostFunction implements StepCostFunction<MapNode, OsmMoveAction> {
        @Override
        public double applyAsDouble(MapNode from, OsmMoveAction action, MapNode to) {
            double distKm = action.getTravelDistance();
            String highwayType = action.getWay().getAttributeValue("highway");
            return distKm * getFunCostFactor(highwayType);
        }
    }

    static class FunHeuristicFunction implements ToDoubleFunction<Node<MapNode, OsmMoveAction>> {
        private final MapNode goalState;

        public FunHeuristicFunction(MapNode goalState) {
            this.goalState = goalState;
        }

        @Override
        public double applyAsDouble(Node<MapNode, OsmMoveAction> node) {
            double sldKm = new Position(node.getState()).getDistKM(goalState);
            return sldKm * MIN_FUN_COST_FACTOR; // admissible: assumes best-case fun factor
        }
    }

    @Override
    protected Problem<MapNode, OsmMoveAction> createProblem(MapNode[] pNodes, OsmMap map,
                                                            MapWayFilter wayFilter, boolean ignoreOneways, int taskSelection) {
        if (taskSelection == 3) // Time (Car)
            return new RouteFindingProblem(pNodes[0], pNodes[1], wayFilter, ignoreOneways, new TimeStepCostFunction());
        else if (taskSelection == 4) // Fun (Cyclist)
            return new RouteFindingProblem(pNodes[0], pNodes[1], wayFilter, ignoreOneways, new FunStepCostFunction());
        else
            return super.createProblem(pNodes, map, wayFilter, ignoreOneways, taskSelection);
    }

    @Override
    protected ToDoubleFunction<Node<MapNode, OsmMoveAction>> createHeuristicFunction(MapNode[] pNodes,
                                                                                     int taskSelection) {
        if (taskSelection == 3) // Time (Car)
            return new TimeHeuristicFunction(pNodes[1]);
        else if (taskSelection == 4) // Fun (Cyclist)
            return new FunHeuristicFunction(pNodes[1]);
        else
            return super.createHeuristicFunction(pNodes, taskSelection);
    }
}
