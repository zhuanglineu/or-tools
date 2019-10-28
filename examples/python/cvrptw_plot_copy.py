# -*- coding: utf-8 -*-
"""
Created on Fri Oct 25 10:39:26 2019
cvrptw_plot
带容量约束和时间窗的vrp问题

@author: Administrator
"""

import os
import numpy as np
from matplotlib import pyplot as plt
from collections import namedtuple
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from datetime import datetime, timedelta


class Customers():
    def __init__(self,
                 extents=None,
                 center=(53.381393, -1.474611),
                 box_size=10,
                 num_stops=100,
                 min_demand=0,
                 max_demand=25,
                 min_tw=1,
                 max_tw=5):
        self.number = num_stops  #顾客数量，含出发点
        Location = namedtuple('Location', ['lat', 'lon'])
        if extents is not None:
            self.extents = extents  #: The lower left and upper right points
            #: Location[lat,lon]: the centre point of the area.
            self.center = Location(
                extents['urcrnrlat'] - 0.5 *
                (extents['urcrnrlat'] - extents['llcrnrlat']),
                extents['urcrnrlon'] - 0.5 *
                (extents['urcrnrlon'] - extents['llcrnrlon']))
        else:
            #: Location[lat,lon]: the centre point of the area.
            (clat, clon) = self.center = Location(center[0], center[1])
            rad_earth = 6367  # km
            circ_earth = np.pi * rad_earth
            #: The lower left and upper right points
            self.extents = {
                'llcrnrlon': (clon - 180 * box_size /
                              (circ_earth * np.cos(np.deg2rad(clat)))),
                'llcrnrlat':
                clat - 180 * box_size / circ_earth,
                'urcrnrlon': (clon + 180 * box_size /
                              (circ_earth * np.cos(np.deg2rad(clat)))),
                'urcrnrlat':
                clat + 180 * box_size / circ_earth
            }
        stops = np.array(range(0, num_stops))
        # 随机产生顾客的坐标
        stdv = 6  # the number of standard deviations 99.9% will be within +-3
        lats = (self.extents['llcrnrlat'] + np.random.randn(num_stops) *
                (self.extents['urcrnrlat'] - self.extents['llcrnrlat']) / stdv)
        lons = (self.extents['llcrnrlon'] + np.random.randn(num_stops) *
                (self.extents['urcrnrlon'] - self.extents['llcrnrlon']) / stdv)
        # 随机产生顾客需求
        demands = np.random.randint(min_demand, max_demand, num_stops)
        
        self.time_horizon = 24*60**2  # A 24 hour period.
        
        # 随机产生顾客时间窗长度
        time_windows =  np.random.random_integers(min_tw * 3600, max_tw * 3600,
                                                  num_stops)
        latest_time = self.time_horizon - time_windows  # 时间窗的最晚开始时间
        start_times = [None for o in time_windows]
        stop_times = [None for o in time_windows]
        # Make random timedeltas, nominaly from the start of the day.
        for idx in range(self.number):
            stime = int(np.random.random_integers(0, latest_time[idx]))
            start_times[idx] = timedelta(seconds=stime)
            stop_times[idx] = (
                start_times[idx] + timedelta(seconds=int(time_windows[idx])))
        
        Customer = namedtuple(
                'Customer',
                [
                    'index',  # the index of the stop
                    'demand',
                    'lat',
                    'lon',
                    'tw_open',
                    'tw_close'
                ])
        self.customers = [
            Customer(idx, dem, lat, lon, tw_open, tw_close)
            for idx, dem, lat, lon, tw_open, tw_close in zip(
                stops, demands, lats, lons, start_times, stop_times)
                ]
        
        self.service_time_per_demand = 300  # seconds
        
    def set_manager(self, manager):
        self.manager = manager
    
    def central_start_node(self, invert=False):
        '''设置depot'''
        num_nodes = len(self.customers)
        dist = np.empty((num_nodes, 1))
        for idx_to in range(num_nodes):
            dist[idx_to] = self._haversine(self.center.lon, self.center.lat,
                                           self.customers[idx_to].lon,
                                           self.customers[idx_to].lat)
        furthest = np.max(dist)

        if invert:
            prob = dist * 1.0 / sum(dist)
        else:
            prob = (furthest - dist * 1.0) / sum(furthest - dist)
        indexes = np.array([range(num_nodes)])
        start_node = np.random.choice(
            indexes.flatten(), size=1, replace=True, p=prob.flatten())
        return start_node[0]
    
    def make_distance_mat(self, method='haversine'):
        '''距离矩阵'''
        self.distmat = np.zeros((self.number, self.number))
        methods = {'haversine': self._haversine}
        assert (method in methods)
        for frm_idx in range(self.number):
            for to_idx in range(self.number):
                if frm_idx != to_idx:
                    frm_c = self.customers[frm_idx]
                    to_c = self.customers[to_idx]
                    self.distmat[frm_idx, to_idx] = self._haversine(
                        frm_c.lon, frm_c.lat, to_c.lon, to_c.lat)
        return self.distmat
    
    def _haversine(self, lon1, lat1, lon2, lat2):
        # convert decimal degrees to radians
        lon1, lat1, lon2, lat2 = map(np.radians, [lon1, lat1, lon2, lat2])

        # haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = (np.sin(dlat / 2)**2 +
             np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2)**2)
        c = 2 * np.arcsin(np.sqrt(a))

        # 6367 km is the radius of the Earth
        km = 6367 * c
        return km
    
    def get_total_demand(self):
        '''所有顾客总需求'''
        return sum([c.demand for c in self.customers])
    
    def return_dist_callback(self, **kwargs):
        '''返回距离回调函数'''
        self.make_distance_mat(**kwargs)
        
        def dist_callback(from_index, to_index):  # test 加上self？
            from_node = self.manager.IndexToNode(from_index)
            to_node = self.manager.IndexToNode(to_index)
            return self.distmat[from_node][to_node]
        return dist_callback
    
    def return_dem_callback(self):
        '''返回需求回调函数'''
        def dem_callback(from_index, to_index):
            from_node = self.manager.IndexToNode(from_index)
            to_node = self.manager.IndexToNode(to_index)
            return self.customers[from_node].demand
        
        return dem_callback
    
    def zero_depot_demand(self, depot):
        '''把depot的需求归零'''
        start_depot = self.customers[depot]
        self.customers[depot] = start_depot._replace(
            demand=0, tw_open=None, tw_close=None)
    
    def make_service_time_callback(self):
        '''返回服务时间回调函数'''
        def service_time_callback(a, b):
            return self.customers[a].demand * self.service_time_per_demand
        
        return service_time_callback
    
    def make_transit_time_callback(self, speed_kmph=10):
        '''返回路程时间回调函数'''
        def transit_time_callback(a, b):
            return self.distmat[a][b] * 1.0 / speed_kmph * 60 ** 2
        
        return transit_time_callback


class Vehicles():
    def __init__(self, capacity=100, cost=100, number=None):
        
        Vehicle = namedtuple('Vehicle', ['index', 'capacity', 'cost'])
        
        if number is None:
            self.number = np.size(capacity)
        else:
            self.number = number
            
        idxs = np.array(range(0, self.number))
        
        if np.isscalar(capacity):
            capacities = capacity * np.ones_like(idxs)
        elif np.size(capacity) != np.size(capacity):
            print('capacity is neither scalar, nor the same size as num!')
        else:
            capacities = capacity

        if np.isscalar(cost):
            costs = cost * np.ones_like(idxs)
        elif np.size(cost) != self.number:
            print(np.size(cost))
            print('cost is neither scalar, nor the same size as num!')
        else:
            costs = cost
        
        self.vehicles = [
            Vehicle(idx, capacity, cost)
            for idx, capacity, cost in zip(idxs, capacities, costs)
                ]
        
    def get_total_capacity(self):
        return sum([v.capacity for v in self.vehicles])
    
    def return_starting_callback(self, customers, sameStartFinish=False):
        '''为每辆车设置不同的出发点和终点'''
        
        self.starts = [
            int(customers.central_start_node()) for o in range(self.number)        
        ]
        
        if sameStartFinish:
            self.ends = self.starts
        else:
            self.ends = [
                int(customers.central_start_node(invert=True))
                for o in range(self.number)
            ]
        
        # 出发点和终点需求置零
        for depot in self.starts:
            customers.zero_depot_demand(depot)
        for depot in self.ends:
            customers.zero_depot_demand(depot)
        
        def start_callback(v):
            return self.starts[v]
        
        return start_callback


def discrete_cmap(N, base_cmap=None):
    """
    Create an N-bin discrete colormap from the specified input map
    """
    # Note that if base_cmap is a string or None, you can simply do
    #    return plt.cm.get_cmap(base_cmap, N)
    # The following works for string, None, or a colormap instance:

    base = plt.cm.get_cmap(base_cmap)
    color_list = base(np.linspace(0, 1, N))
    cmap_name = base.name + str(N)
    return base.from_list(cmap_name, color_list, N)

    
def vehicle_output_string(manager, routing, plan):
    '''输出每个车辆的求解路径'''
    dropped = []
    for order in range(routing.Size()):
        if (plan.Value(routing.NextVar(order)) == order):
            dropped.append(str(order))
    
    capacity_dimension = routing.GetDimensionOrDie('capacity')
    time_dimension = routing.GetDimensionOrDie('time')
    plan_output = ''
    
    for route_number in range(routing.vehicles()):  # routing.vehicles求解后的车辆路径数量
        order = routing.Start(route_number)
        plan_output += 'Route {0}:'.format(route_number)
        if routing.IsEnd(plan.Value(routing.NextVar(order))):
            plan_output += ' Empty\n'
        else:
            while True:
                load_var = capacity_dimension.CumulVar(order)
                time_var = time_dimension.CumulVar(order)
                node = manager.IndexToNode(order)
                plan_output = \
                    ' {node} Load({load}) Time({tmin}, {tmax}) -> '.format(
                        node=node,
                        load=plan.Value(load_var),
                        tmin=str(timedelta(seconds=plan.Min(time_var))),
                        tmax=str(timedelta(seconds=plan.Max(time_var))))
                if routing.IsEnd(order):
                    plan_output += ' EndRoute {0}. \n'.format(route_number)
                    break
                order = plan.Value(routing.NextVar(order))
        plan_output += '\n'
    return (plan_output, dropped)


def build_vehicle_route(manager, routing, plan, customers, veh_number):
    veh_used = routing.IsVehicleUsed(plan, veh_number)
    print('Vehicle {0} is used {1}'.format(veh_number, veh_used))
    if veh_used:
        route = []
        node = routing.Start(veh_number)  # Get the starting node index
        route.append(customers.customers[manager.IndexToNode(node)])
        while not routing.IsEnd(node):
            route.append(customers.customers[manager.IndexToNode(node)])
            node = plan.Value(routing.NextVar(node))

        route.append(customers.customers[manager.IndexToNode(node)])
        return route
    else:
        return None
    

def plot_vehicle_routes(veh_route, ax1, customers, vehicles):
    """
    Plot the vehicle routes on matplotlib axis ax1.

    Args: veh_route (dict): a dictionary of routes keyed by vehicle idx.  ax1
    (matplotlib.axes._subplots.AxesSubplot): Matplotlib axes  customers
    (Customers): the customers instance.  vehicles (Vehicles): the vehicles
    instance.
  """
    veh_used = [v for v in veh_route if veh_route[v] is not None]

    cmap = discrete_cmap(vehicles.number + 2, 'nipy_spectral')

    for veh_number in veh_used:

        lats, lons = zip(*[(c.lat, c.lon) for c in veh_route[veh_number]])
        lats = np.array(lats)
        lons = np.array(lons)
        s_dep = customers.customers[vehicles.starts[veh_number]]
        s_fin = customers.customers[vehicles.ends[veh_number]]
        ax1.annotate(
            'v({veh}) S @ {node}'.format(
                veh=veh_number, node=vehicles.starts[veh_number]),
            xy=(s_dep.lon, s_dep.lat),
            xytext=(10, 10),
            xycoords='data',
            textcoords='offset points',
            arrowprops=dict(
                arrowstyle='->',
                connectionstyle='angle3,angleA=90,angleB=0',
                shrinkA=0.05),
        )
        ax1.annotate(
            'v({veh}) F @ {node}'.format(
                veh=veh_number, node=vehicles.ends[veh_number]),
            xy=(s_fin.lon, s_fin.lat),
            xytext=(10, -20),
            xycoords='data',
            textcoords='offset points',
            arrowprops=dict(
                arrowstyle='->',
                connectionstyle='angle3,angleA=-90,angleB=0',
                shrinkA=0.05),
        )
        ax1.plot(lons, lats, 'o', mfc=cmap(veh_number + 1))
        ax1.quiver(
            lons[:-1],
            lats[:-1],
            lons[1:] - lons[:-1],
            lats[1:] - lats[:-1],
            scale_units='xy',
            angles='xy',
            scale=1,
            color=cmap(veh_number + 1))


def main():
    # 顾客
    customers = Customers(
        num_stops=50,
        min_demand=1,
        max_demand=15,
        box_size=40,
        min_tw=3,
        max_tw=6)
    
    # 车辆
    capacity = [50, 75, 100, 125, 150, 175, 200, 250]
    cost = [int(100 + 2 * np.sqrt(c)) for c in capacity]
    vehicles = Vehicles(capacity=capacity, cost=cost)
    
    assert (customers.get_total_demand() < vehicles.get_total_capacity())
    
    start_fn = vehicles.return_starting_callback(customers,
                                                 sameStartFinish=False)
    
    manager = pywrapcp.RoutingIndexManager(
        customers.number,
        vehicles.number,
        vehicles.starts,
        vehicles.ends)
    customers.set_manager(manager)
    
    model_parameters = pywrapcp.DefaultRoutingModelParameters()
    routing = pywrapcp.RoutingModel(manager, model_parameters)
    
    parameters = pywrapcp.DefaultRoutingSearchParameters()
    parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    parameters.local_search_operators.use_tsp_opt = pywrapcp.BOOL_FALSE
    parameters.local_search_operators.use_path_lns = pywrapcp.BOOL_FALSE
    parameters.local_search_operators.use_inactive_lns = pywrapcp.BOOL_FALSE
    parameters.time_limit.seconds = 10
    parameters.use_full_propagation = True
    
    dist_fn = customers.return_dist_callback()
    dist_fn_index = routing.RegisterTransitCallback(dist_fn)
    
    dem_fn = customers.return_dem_callback()
    dem_fn_index  = routing.RegisterTransitCallback(dem_fn)
    
    service_time_fn = customers.make_service_time_callback()
    transit_time_fn = customers.make_transit_time_callback()
    def tot_time_fn(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return service_time_fn(from_node, to_node) + transit_time_fn(from_node, to_node)
    tot_time_fn_index = routing.RegisterTransitCallback(tot_time_fn)
    
    # 目标
    routing.SetArcCostEvaluatorOfAllVehicles(dist_fn_index)
    for veh in vehicles.vehicles:
        routing.SetFixedCostOfVehicle(veh.cost, int(veh.index))
    
    null_capacity_slack = 0
    routing.AddDimensionWithVehicleCapacity(
        dem_fn_index,
        null_capacity_slack,
        capacity,
        True,
        'capacity')
    
    routing.AddDimension(
        tot_time_fn_index,
        customers.time_horizon,
        customers.time_horizon,
        True,
        'time')
    
    time_dimension = routing.GetDimensionOrDie('time')
    for cust in customers.customers:
        if cust.tw_open is not None:
            time_dimension.CumulVar(manager.NodeToIndex(cust.index)).SetRange(
                cust.tw_open.seconds, cust.tw_close.seconds)
    
    # To add disjunctions just to the customers, make a list of non-depots.
    non_depot = set(range(customers.number))
    non_depot.difference_update(vehicles.starts)
    non_depot.difference_update(vehicles.ends)
    penalty = 400000  # The cost for dropping a node from the plan.
    nodes = [routing.AddDisjunction([manager.NodeToIndex(c)], penalty) for c in non_depot]
    
    assignment = routing.SolveWithParameters(parameters)
    
    if assignment:
        print('The Objective Value is {0}'.format(assignment.ObjectiveValue()))
        

        plan_output, dropped = vehicle_output_string(manager, routing, assignment)
        print(plan_output)
        print('dropped nodes: ' + ', '.join(dropped))

        # you could print debug information like this:
        # print(routing.DebugOutputAssignment(assignment, 'Capacity'))

        vehicle_routes = {}
        for veh in range(vehicles.number):
            vehicle_routes[veh] = build_vehicle_route(manager, routing, assignment,
                                                      customers, veh)

        # Plotting of the routes in matplotlib.
        fig = plt.figure()
        ax = fig.add_subplot(111)
        # Plot all the nodes as black dots.
        clon, clat = zip(*[(c.lon, c.lat) for c in customers.customers])
        ax.plot(clon, clat, 'k.')
        # plot the routes as arrows
        plot_vehicle_routes(vehicle_routes, ax, customers, vehicles)
        plt.show()

    else:
        print('No assignment')


if __name__ == '__main__':
    main()
    
    
    
    
    
        
    
    
    
        





