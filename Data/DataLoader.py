import numpy as np

class DataLoader:

    def __init__(self, nodes_num=1000, seed=30):
        self.nodes_num = nodes_num
        self.seed = seed
        self.data = self.create_data_model()

    def create_data_model(self):
        """Stores the data for the problem."""
        data = {}
        np.random.seed(self.seed)

        # Generate a random distance matrix
        x = np.random.randint(1, 100, (self.nodes_num, self.nodes_num))
        x = np.triu(x)
        x += x.T - np.diag(x.diagonal())

        # Set the diagonal to zero
        np.fill_diagonal(x, 0)

        data["distance_matrix"] = x.tolist()
        data["num_vehicles"] = self.nodes_num
        data["depot"] = 0
        data["demands"] = [0] + np.random.randint(1, 100, [self.nodes_num - 1]).tolist()
        data["vehicle_capacities"] = [max(data["demands"]) + 50 for _ in range(self.nodes_num)]

        return data

    def get_data(self):
        return self.data
