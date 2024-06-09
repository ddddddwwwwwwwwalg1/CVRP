from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

class Ortools:
    def __init__(self, data):
        self.data = data
        self.manager = pywrapcp.RoutingIndexManager(
            len(self.data["distance_matrix"]), self.data["num_vehicles"], self.data["depot"]
        )
        self.routing = pywrapcp.RoutingModel(self.manager)
        self.solution = None

    def create_routing_model(self):
        # Create and register a transit callback.
        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = self.manager.IndexToNode(from_index)
            to_node = self.manager.IndexToNode(to_index)
            return self.data["distance_matrix"][from_node][to_node]

        transit_callback_index = self.routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        self.routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Capacity constraint.
        def demand_callback(from_index):
            """Returns the demand of the node."""
            # Convert from routing variable Index to demands NodeIndex.
            from_node = self.manager.IndexToNode(from_index)
            return self.data["demands"][from_node]

        demand_callback_index = self.routing.RegisterUnaryTransitCallback(demand_callback)
        self.routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,  # null capacity slack
            self.data["vehicle_capacities"],  # vehicle maximum capacities
            True,  # start cumul to zero
            "Capacity",
        )

    def solve(self):
        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        search_parameters.time_limit.FromSeconds(90)

        # Solve the problem.
        self.solution = self.routing.SolveWithParameters(search_parameters)

    def print_solution(self):
        """Prints solution on console."""
        if not self.solution:
            print("No solution found!")
            return

        # print(f"Objective: {self.solution.ObjectiveValue()}")
        total_distance = 0
        total_load = 0
        for vehicle_id in range(self.data["num_vehicles"]):
            index = self.routing.Start(vehicle_id)
            plan_output = f"Route for vehicle {vehicle_id}:\n"
            route_distance = 0
            route_load = 0
            while not self.routing.IsEnd(index):
                node_index = self.manager.IndexToNode(index)
                route_load += self.data["demands"][node_index]
                plan_output += f" {node_index} Load({route_load}) -> "
                previous_index = index
                index = self.solution.Value(self.routing.NextVar(index))
                route_distance += self.routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id
                )
            plan_output += f" {self.manager.IndexToNode(index)} Load({route_load})\n"
            plan_output += f"Distance of the route: {route_distance}m\n"
            plan_output += f"Load of the route: {route_load}\n"
            print(plan_output)
            total_distance += route_distance
            total_load += route_load
        # print(f"Total distance of all routes: {total_distance}m")
        # print(f"Total load of all routes: {total_load}")
        return total_distance
