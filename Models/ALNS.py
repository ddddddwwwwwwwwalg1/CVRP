import copy
import math
import time
import random
import numpy as np
import pandas as pd
from numpy.random import choice


class ALNS:

    def __init__(self, data):
        self.data = data
        # 模拟退火温度
        self.T = 100
        # 降温速度
        self.a = 0.97
        # destroy的城市数量
        self.destroy_nodes_cnt = int(len(self.data["distance_matrix"]) * 0.1)
        # destroy算子的初始权重
        self.destroy_w = [1, 1, 1]
        # repair算子的初始权重
        self.repair_w = [1, 1]
        # destroy算子的使用次数
        self.destroy_cnt = [0, 0, 0]
        # repair算子的使用次数
        self.repair_cnt = [0, 0]
        # destroy算子的初始得分
        self.destroy_score = [1, 1, 1]
        # repair算子的初始得分
        self.repair_score = [1, 1]
        # destroy和repair的挥发系数
        self.lambda_rate = 0.5
        self.max_iter = 200

    # 计算VRP距离
    def dis_cal(self, routes):
        dist_mat = self.data["distance_matrix"]
        distance = 0
        for r in routes:
            if len(r) > 0:
                for i in range(len(r) - 1):
                    distance += dist_mat[r[i]][r[i + 1]]
                distance += dist_mat[r[-1]][r[0]]
        return distance

    # 随机删除算子
    def random_destroy(self, x):
        new_x = copy.deepcopy(x)
        removed_nodes = []
        while len(removed_nodes) < self.destroy_nodes_cnt:
            ## 随机选择一个route
            removed_r = new_x[choice(range(len(new_x)))]
            ## 原解中remove这个route
            new_x.remove(removed_r)
            ## r的一个随机节点
            node = choice(removed_r[1:-1])
            removed_nodes.append(node)
            removed_r.remove(node)

            if len(removed_r) > 2:
                new_x.append(removed_r)
        return removed_nodes, new_x

    # 删除距离最大的N个节点,贪心破坏算子
    def max_d_destroy(self, x):
        new_x = copy.deepcopy(x)
        removed_nodes = []
        while len(removed_nodes) < self.destroy_nodes_cnt:
            best_fitness = 0
            route, node = 0, 0
            for r in new_x:
                for n in r[1:-1]:
                    if len(r) <= 3:
                        fitness = self.dis_cal([r])
                    else:
                        new_r = copy.deepcopy(r)
                        new_r.remove(n)
                        fitness = self.dis_cal([r]) - self.dis_cal([new_r])
                    if fitness > best_fitness:
                        best_fitness = fitness
                        route = r
                        node = n
            removed_nodes.append(node)
            new_x.remove(route)
            route.remove(node)
            if len(route) > 2:
                new_x.append(route)
        return removed_nodes, new_x

    # 剔除相关度高的节点
    def sim_destroy(self, x):
        new_x = copy.deepcopy(x)
        removed_nodes = []
        dist_mat = self.data["distance_matrix"]
        route = new_x[choice(range(len(new_x)))]
        node = choice(route[1:-1])
        while len(removed_nodes) < self.destroy_nodes_cnt:
            new_x.remove(route)
            removed_nodes.append(node)
            route.remove(node)
            if len(route) > 2:
                new_x.append(route)
            min_relate = float('inf')
            for r in new_x:
                for n in r[1:-1]:
                    next_node = n
                    fitness = dist_mat[node][next_node]
                    if fitness < min_relate:
                        min_relate = fitness
                        route = r
                        node = n
        return removed_nodes, new_x

    # destroy操作
    def destroy(self, flag, x):
        removed_nodes = []
        new_x = []
        if flag == 0:
            removed_nodes, new_x = self.random_destroy(x)
        elif flag == 1:
            removed_nodes, new_x = self.max_d_destroy(x)
        elif flag == 2:
            removed_nodes, new_x = self.sim_destroy(x)
        return removed_nodes, new_x

    # 判断是否满足业务约束
    def if_violation(self, route):
        load = 0
        for node in route:
            load += self.data["demands"][node]
        return load <= self.data["vehicle_capacities"][0]

    # 选择destroy算子
    def select_and_destroy(self, x):
        # 轮盘赌逻辑选择算子
        prob = self.destroy_w / np.array(self.destroy_w).sum()
        seq = [i for i in range(len(self.destroy_w))]
        destroy_operator = choice(seq, size=1, p=prob)[0]
        # destroy操作
        removed_nodes, test_x = self.destroy(destroy_operator, x)
        return removed_nodes, test_x, destroy_operator

    # 随机插入
    def random_insert(self, x, removed_nodes):
        new_x = copy.deepcopy(x)
        dist_mat = self.data["distance_matrix"]
        demand_list = self.data["demands"]
        cap = self.data["vehicle_capacities"][0]
        while removed_nodes:
            insert_node = removed_nodes.pop(0)
            bestFitness = float('inf')
            if len(new_x) == 0:
                if demand_list[insert_node] <= cap:
                    new_x.append([0] + [insert_node] + [0])
                    continue
                else:
                    continue
            routeNr = random.randint(1, len(new_x))
            routeList = random.sample(range(len(new_x)), routeNr)

            bestRouteIndex = 0
            bestNodeIndex = 0
            for i in routeList:
                tmpRoute = copy.deepcopy(new_x[i])
                ## 除了第一位都可以，所以可以最多可以有n-1个节点
                nodeNr = random.randint(1, len(tmpRoute) - 1)
                nodeList = random.sample(range(1, len(tmpRoute)), nodeNr)
                ## 插入在j的前面
                for j in nodeList:
                    tmpRoute = tmpRoute[:j] + [insert_node] + tmpRoute[j:]
                    demand_sum = 0
                    for n in tmpRoute:
                        demand_sum += demand_list[n]
                    if demand_sum <= cap:
                        node0 = tmpRoute[j - 1]
                        node1 = tmpRoute[j + 1]
                        fitness = dist_mat[node0][insert_node] + dist_mat[insert_node][node1] - dist_mat[node0][node1]
                        if fitness < bestFitness:
                            bestFitness = fitness
                            bestRouteIndex = i
                            bestNodeIndex = j

            if bestFitness < float('inf'):
                route = new_x[bestRouteIndex]
                new_x.remove(route)
                new_route = route[:bestNodeIndex] + [insert_node] + route[bestNodeIndex:]
                new_x.append(new_route)
            else:
                if demand_list[insert_node] <= cap:
                    new_route = [0] + [insert_node] + [0]
                    new_x.append(new_route)
        return new_x

    # 贪心插入
    def greedy_insert(self, x, removed_nodes):

        new_x = copy.deepcopy(x)
        dist_mat = self.data["distance_matrix"]
        demand_list = self.data["demands"]
        cap = self.data["vehicle_capacities"][0]
        while removed_nodes:
            insert_node = removed_nodes.pop(0)

            if len(new_x) == 0:
                if demand_list[insert_node] <= cap:
                    new_x.append([0] + [insert_node] + [0])
                    continue
                else:
                    continue
            bestFitness = float('inf')
            best_r = []
            best_new_r = []
            for r in new_x:
                for n in r[1:-1]:
                    index = r.index(n)
                    new_r = r[:index] + [insert_node] + r[index:]

                    demand_sum = 0
                    for i in new_r:
                        demand_sum += demand_list[i]
                    if demand_sum <= cap:
                        node0 = new_r[index - 1]
                        node1 = new_r[index + 1]
                        fitness = dist_mat[node0][insert_node] + dist_mat[insert_node][node1] - dist_mat[node0][node1]
                        if fitness < bestFitness:
                            bestFitness = fitness
                            best_r = r
                            best_new_r = new_r

            if bestFitness < float('inf'):
                new_x.remove(best_r)
                new_x.append(best_new_r)
            else:
                if demand_list[insert_node] <= cap:
                    new_x.append([0] + [insert_node] + [0])
        return new_x

    # repair操作
    def repair(self, flag, x, removed_nodes):
        # 两个repair算子，第一个是随机插入，第二个贪心插入
        if flag == 0:
            return self.random_insert(x, removed_nodes)
        elif flag == 1:
            return self.greedy_insert(x, removed_nodes)

    # 选择repair算子
    def select_and_repair(self, x, removed_nodes):
        # 轮盘赌逻辑选择算子
        prob = self.repair_w / np.array(self.repair_w).sum()
        seq = [i for i in range(len(self.repair_w))]
        repair_operator = choice(seq, size=1, p=prob)[0]
        # repair操作：此处设定repair_operator=1，即只使用贪心策略
        test_x = self.repair(repair_operator, x, removed_nodes)
        return test_x, repair_operator

    def run(self):
        ## 当前解，第一代，贪心策略生成
        removed_nodes = [i for i in range(1, np.array(self.data["distance_matrix"]).shape[0])]
        x = self.repair(1, [], removed_nodes)
        # 历史最优解，第一代和当前解相同，注意是深拷贝，此后有变化不影响x，也不会因x的变化而被影响
        history_best_x = copy.deepcopy(x)

        # 迭代
        cur_iter = 0
        print('cur_iter: {}, best_f: {}'.format(cur_iter, self.dis_cal(history_best_x)))
        while cur_iter < self.max_iter:
            # destroy算子
            remove_nodes, test_x0, destroy_operator_index = self.select_and_destroy(x)
            self.destroy_cnt[destroy_operator_index] += 1

            # repair算子
            test_x1, repair_operator_index = self.select_and_repair(test_x0, remove_nodes)
            self.repair_cnt[repair_operator_index] += 1

            if self.dis_cal(test_x1) <= self.dis_cal(x):
                # 测试解更优，更新当前解
                x = copy.deepcopy(test_x1)
                if self.dis_cal(test_x1) <= self.dis_cal(history_best_x):
                    # 测试解为历史最优解，更新历史最优解，并设置最高的算子得分
                    history_best_x = copy.deepcopy(test_x1)
                    self.destroy_score[destroy_operator_index] = 1.5
                    self.repair_score[repair_operator_index] = 1.5
                else:
                    # 测试解不是历史最优解，但优于当前解，设置第二高的算子得分
                    self.destroy_score[destroy_operator_index] = 1.2
                    self.repair_score[repair_operator_index] = 1.2
            else:
                if np.random.random() < np.exp((self.dis_cal(x) - self.dis_cal(test_x1)) / self.T):
                    # 当前解优于测试解，但满足模拟退火逻辑，依然更新当前解，设置第三高的算子得分
                    x = copy.deepcopy(test_x1)
                    self.destroy_score[destroy_operator_index] = 0.8
                    self.repair_score[repair_operator_index] = 0.8
                else:
                    # 当前解优于测试解，也不满足模拟退火逻辑，不更新当前解，设置最低的算子得分
                    self.destroy_score[destroy_operator_index] = 0.5
                    self.repair_score[repair_operator_index] = 0.5

                    # 更新destroy算子的权重
            self.destroy_w[destroy_operator_index] = \
                self.destroy_w[destroy_operator_index] * self.lambda_rate + \
                (1 - self.lambda_rate) * self.destroy_score[destroy_operator_index] / self.destroy_cnt[destroy_operator_index]
            # 更新repair算子的权重
            self.repair_w[repair_operator_index] = \
                self.repair_w[repair_operator_index] * self.lambda_rate + \
                (1 - self.lambda_rate) * self.repair_score[repair_operator_index] / self.repair_cnt[repair_operator_index]
            # 降低温度
            self.T = self.a * self.T
            # 结束一轮迭代，重置模拟退火初始温度
            cur_iter += 1
            if cur_iter % 10 == 0:
                print('cur_iter: {}, best_f: {}'.format(cur_iter, self.dis_cal(history_best_x)))

        return self.dis_cal(history_best_x)