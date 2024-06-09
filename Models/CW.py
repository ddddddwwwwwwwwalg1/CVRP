import numpy as np
from operator import itemgetter

class CW:
    def __init__(self, data):

        self.data = data
        self.routes = []

    def savings_algorithm(self):
        """Implements the savings algorithm for CVRP."""
        from operator import itemgetter

        saving = 0
        dists_matrix = self.data["distance_matrix"]
        savings = []

        # Initialize routes
        for i in range(1, self.data["num_vehicles"]):
            self.routes.append([i])

        for i in range(1, len(self.routes) + 1):  # Calculate savings Sij = Ci0 + C0j - Cij
            for j in range(1, len(self.routes) + 1):
                if i == j:
                    pass
                else:
                    saving = (dists_matrix[i][0] + dists_matrix[0][j]) - dists_matrix[i][j]
                    savings.append([i, j, saving])  # Store results as tuples in a list

        savings = sorted(savings, key=itemgetter(2), reverse=True)  # Sort by savings in descending order

        for i in range(len(savings)):
            startRoute = []
            endRoute = []
            routeDemand = 0

            # Only expose the first and last nodes
            for j in range(len(self.routes)):
                # Link two different j
                if savings[i][0] == self.routes[j][-1]:
                    endRoute = self.routes[j]
                elif savings[i][1] == self.routes[j][0]:
                    startRoute = self.routes[j]

                if (len(startRoute) != 0) and (len(endRoute) != 0):
                    for k in range(len(startRoute)):
                        routeDemand += self.data["demands"][startRoute[k]]
                    for k in range(len(endRoute)):
                        routeDemand += self.data["demands"][endRoute[k]]

                    routeDistance = 0
                    routestore = [0] + endRoute + startRoute + [0]
                    for i in range(len(routestore) - 1):
                        routeDistance += dists_matrix[routestore[i]][routestore[i + 1]]

                    if routeDemand <= self.data["vehicle_capacities"][0]:  # Modify routes according to constraints
                        self.routes.remove(startRoute)
                        self.routes.remove(endRoute)
                        self.routes.append(endRoute + startRoute)
                    break

        # Insert 0 at the start and end
        for i in range(len(self.routes)):
            self.routes[i].insert(0, 0)
            self.routes[i].insert(len(self.routes[i]), 0)

        return self.routes

    def print_routes(self):
        """Prints the routes and their costs and demands."""
        for route in self.routes:
            costs = 0
            demands = 0
            for j in range(len(route) - 1):
                costs += self.data["distance_matrix"][route[j]][route[j + 1]]
                demands += self.data["demands"][route[j]]
            demands += self.data["demands"][route[-1]]
            print(f"Route: {route}  Distance: {costs}  Load: {demands}")

    def calc_costs(self, routes):
        """Calculates and prints the total cost of the routes."""
        cost = 0
        for route in routes:
            for j in range(len(route) - 1):
                cost += self.data["distance_matrix"][route[j]][route[j + 1]]
        return cost

    def print_solution(self):
        cw_routes = self.savings_algorithm()
        return self.calc_costs(cw_routes)
