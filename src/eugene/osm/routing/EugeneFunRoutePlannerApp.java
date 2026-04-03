package eugene.osm.routing;

import aimax.osm.gui.fx.applications.RoutePlannerOsmApp;
import aimax.osm.routing.RouteCalculator;

public class EugeneFunRoutePlannerApp extends RoutePlannerOsmApp {

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public String getTitle() {
        return "Eugene's fun Route Planner";
    }

    @Override
    protected RouteCalculator createRouteCalculator() {
        return new FunExtendedRouteCalculator(); //return mine instead
    }
}
