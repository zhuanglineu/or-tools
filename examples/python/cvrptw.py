# -*- coding: utf-8 -*-
"""
Created on Mon Oct 28 09:48:38 2019
cvrptw
用Google OR-Tools搭建简单车辆路线规划问题
https://blog.csdn.net/yuuyuhaksho/article/details/87536608
代码不全
@author: Administrator
"""


# orTools里cost的计算是通过callback函数实现的，函数的格式是function(from_node_index, to_node_index)。
# 在生成约束的时候把函数直接作为变量传递给orTools的类。
class CreateDistanceEvaluator():
    def __init__(self, locationData, multiplier=1.3):
        self.distances = {}
        self._multiplier = multiplier 
        
        for from_node in np.arange(len(locationData)):
            self.distances[from_node] = {}
            for to_node in np.arange(len(locationData)):
                if np.alltrue(np.equal(from_node, to_node)):
                    self.distances[from_node][to_node] = 0
                else:
                    self.distances[from_node][to_node] = convertToInt(
                            self.getDistanceInKm(locationData[from_node],
                            locationData[to_node]) * self._multiplier)
    
    @staticmethod
    def getDistanceInKm(coord1,coord2):
        # https://en.wikipedia.org/wiki/Haversine_formula
        lat1, lon1 = coord1
        lat2, lon2 = coord2
        
        if np.isnan(lat1 * lon1 * lat2 * lon2):
            return 0
        
        def deg2rad(deg):
            return deg * (math.pi / 180)
        
        R = 6371  # 地球半径（公里）
        dLat = deg2rad(lat2-lat1)  
        dLon = deg2rad(lon2-lon1); 
        a = (   math.sin(dLat/2) * math.sin(dLat/2) 
                + math.cos(deg2rad(lat1)) * math.cos(deg2rad(lat2)) * 
                  math.sin(dLon/2) * math.sin(dLon/2)    )
    
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a)) 
        d = R * c 
        return d
    
    def distance_evaluator(self, from_node, to_node):
        """
        callback函数。在计算cost的过程中直接抽取预先计算的距离值，提高速度。
        """
        return self.distances[from_node][to_node]


class CreateDemandEvaluator():
    def __init__(self, demandData):
        self._demands = demandData
        
    def demand_evaluator(self, from_node, to_node):
        """
        Callback函数
        """
        del to_node
        return self._demands[from_node]


# 计算时间cost的callback函数。
# 由于每辆车的速度不同，计算cost的方式有区别，需要生成一组不同的evaluator类。
class CreateAllTransitEvaluators():
    def __init__(self, vehicles, distances, serviceTimes):
        """
        callback函数list
        """
        self._vehicles = vehicles
        self._distances = distances
        self._serviceTimes = serviceTimes
        self.evaluators = []
        # 每辆车根据速度不同单独生成对应的evaluator:
        for v in vehicles.speeds:
            evaluator = CreateOneTransitEvaluator(v, self._distances, 
                                self._serviceTimes).one_transit_evaluator
            self.evaluators.append(evaluator)


class CreateOneTransitEvaluator():
    def __init__(self, speed, distances, serviceTimes):
        self._speed = speed
        self._distances = distances
        self._serviceTimes = serviceTimes
        
    def one_transit_evaluator(self, from_node, to_node):
        """
        单一callback函数：
        计算单个节点的时间总量 = 当前节点到下一节点的距离 / 车辆速度 + 当前节点的服务时长
        """
        if from_node == to_node:
            return 0
        if self._speed == 0:
            return sys.maxsize
        return convertToInt(self._distances[from_node][to_node] / self._speed 
                + self._serviceTimes[from_node])

'''
定义约束：RoutingModel类的接口定义

其中AddDimension(evaluator, slack, capacity, *args)用来定义约束，常用有4种：

AddDimension(singleEvaluator, intSlack, intCapacity, *args): 所有车辆共用callback函数
evaluator 和约束上限capacity。
AddDimensionWithVehicleCapacity(singleEvaluator, intSlack, listCapacity, *args): 所有车辆
共用evaluator，但使用自定义的约束上限capacity，入参格式为list。比如：每辆车的容量
不同。
AddDimensionWithVehicleTransits(listEvaluator, intSlack, intCapacity, *args): 所有车辆共用
约束上限capacity，但使用自定义的cost计算方式，入参格式为evaluator list, 对应不同车辆。
比如：每辆车的平均驾驶速度不同，计算路途时间需要不同的evaluator。
AddDimensionWithVehicleTransitAndCapacity(listEvaluator, intSlack, listCapacity, *args):
cost计算方式和约束上限都可以用每辆车的自定义方式。
另外：

第二项int 参数slack的定义规定了约束变量的松弛范围。
第四项boolean 参数fix_start_cumul_to_zero定义了cost是否默认从0开始计数。当涉及时间窗口，time cost的起始点大于0，这一项需要设置为false.
第五项string 参数name为这个约束维度命名，方便在优化结果中提取约束相关的各项指标。
'''

def add_capacity_constraints(routing, listOfConstraints, evaluator, varName):
    
    name = varName
    routing.AddDimensionWithVehicleCapacity(
           evaluator, 
           0, 
           listOfConstraints, 
           True, 
           name)


def add_transit_and_capacity_constraints(routing, listOfConstraints, 
                                         listOfEvaluators, intSlack, varName):
    name = varName
    routing.AddDimensionWithVehicleTransitAndCapacity(
        listOfEvaluators,
        intSlack,
        listOfConstraints,
        False,
        name)


'''
定义时间窗口约束：

RoutingModel.GetDimensionOrDie函数根据给定的约束维度名称提取对应的约束信息。

从dimension里返回的值有几类：

transit变量：当前节点的增加或减少值，比如时间约束里就是这个节点耗费的服务时间和路途时间
cumulative 变量：从开始节点到当前节点的累加量，比如时间约束里就是车辆到达这个节点以前耗费的总时长。
slack变量
时间窗口的约束就是加在每个cumulative 变量上的，所以需要首先调用RoutingModel.GetDimensionOrDie获取CumulVar的值。

另外注意几点细节：

数据的顺序角标有两种，一种是输入顺序，对应第一章里以i 标识的变量，在程序中是node_idx和veh_idx；另外一种是在每个路线中的顺序，对应第一章里以
vi v_i
v 
i
​	
 
标识的变量，在程序中是index变量。而函数NodeToIndex就是从自然顺序到路线顺序的lookup过程。
orTools里RoutingModel.NodeToIndex和RoutingModel.Start/End 函数的区别：所有代表客户站点的节点（就是路线中的非起始或终止节点）可以直接通过NodeToIndex获取；而所有的起始和终止节点由于可能被多个车辆引用，所以在分配节点编号时并不对应自然顺序。要获取这些节点，必须用Start或End函数。
RoutingModel.AddToAssignment的作用：assignment指经过求解器solve的路径规划结果。通过assignment的各种函数（比如Value, Min, Max）可以获取模型变量的取值或取值范围。对于约束维度来说，slackVar变量默认并不保存在最终的assignment里，所以如果想在最终结果里看到时间变量的松弛范围，需要在约束建模时调用AddToAssignment(SlackVar)。
任何约束维度的Slack变量都只存在于每段弧的起始节点上，所以如果试图对end节点调用SlackVar(RoutingModel.End(vehicle_id)，程序会报错。
'''

def add_timewindow_constraints(routing, data, varName='time'):
    
    time_dimension = routing.GetDimensionOrDie(varName)
    for node_idx, time_window in enumerate(data.timeWindows):
        if node_idx <= np.max(data.depotIndex):
            continue
        index = routing.NodeToIndex(node_idx)
        servTime = data.serviceTimes[node_idx]
        time_dimension.CumulVar(index).SetRange(
                                    time_window[0], time_window[1]-servTime)
        routing.AddToAssignment(time_dimension.SlackVar(index))
    for veh_idx in np.arange(data.nrVehicles):
        index = routing.Start(veh_idx)
        servTime = data.serviceTimes[data.depotIndex[veh_idx]]
        time_dimension.CumulVar(index).SetRange(
                                        data.earliestWorkHours[veh_idx],
                                        data.latestWorkHours[veh_idx]-servTime)
        routing.AddToAssignment(time_dimension.SlackVar(index))
    for veh_idx in np.arange(len(data.depotIndex)):
        index = routing.End(veh_idx)
        servTime = data.serviceTimes[data.depotIndex[veh_idx]]
        time_dimension.CumulVar(index).SetRange(
                                        data.earliestWorkHours[veh_idx],
                                        data.latestWorkHours[veh_idx]-servTime)


class ConsolePrinter():
    def __init__(self, data, routing, assignment, distances):
        self._data = data
        self._routing = routing
        self._assignment = assignment
        self._distances = distances
        
    def printAll(self):
        total_dist = 0
        total_siteCount = 0
        total_fulfilledDemand = 0
        capacity_dimension = self._routing.GetDimensionOrDie('capacity')
        distance_dimension = self._routing.GetDimensionOrDie('dailyDistance')
        time_dimension = self._routing.GetDimensionOrDie('time')
        siteCount_dimension = self._routing.GetDimensionOrDie('dailyNrJobs')
        
        for vehicle_id in np.arange(self._data.nrVehicles):
            index = self._routing.Start(vehicle_id)
            plan_output = 'Route for person {0}: \n'.format(vehicle_id)
            route_startTime = self._assignment.Value(time_dimension.CumulVar(index))
            route_serviceTime = 0
            route_timeWindow = []
            while not self._routing.IsEnd(index):
                node_index = self._routing.IndexToNode(index)
                next_node_index = self._routing.IndexToNode(
                        self._assignment.Value(self._routing.NextVar(index)))
                step_dist = self._distances[node_index][next_node_index]
                step_load = self._data.demands[node_index]
                step_serviceTime = self._data.serviceTimes[node_index]
                route_serviceTime += step_serviceTime
                step_timewindow = self._data.timeWindows[node_index]
                route_timeWindow.append(step_timewindow)
                time_var = time_dimension.CumulVar(index)
                time_min = self._assignment.Min(time_var)
                time_max = self._assignment.Max(time_var)
                slack_var = time_dimension.SlackVar(index)
                slack_min = self._assignment.Min(slack_var)
                slack_max = self._assignment.Max(slack_var)
                
                plan_output += (
                    ' {node} capacity({capa}) distance({dist}) serviceTime({minTime},{maxTime}) slack({minSlack},{maxSlack})->\n'
                    .format(node=node_index, capa=step_load, dist=step_dist, 
                        minTime=time_min, maxTime=time_max, minSlack=slack_min, 
                        maxSlack=slack_max) )
                index = self._assignment.Value(self._routing.NextVar(index))
            
            end_idx = self._routing.End(vehicle_id) 
            route_endTime = self._assignment.Value(time_dimension.CumulVar(end_idx)) 
            route_dist = self._assignment.Value(distance_dimension.CumulVar(end_idx))
            route_load = self._assignment.Value(capacity_dimension.CumulVar(end_idx))
            route_siteCount = self._assignment.Value(siteCount_dimension.CumulVar(end_idx))
            node_index = self._routing.IndexToNode(index)
            total_dist += route_dist
            total_siteCount += route_siteCount
            total_fulfilledDemand += route_load
            
            plan_output += ' {0} \n'.format(node_index)
            plan_output += ('Objective: minimize vehicle cost + distance cost, maximize number of sites visited\nConstraint:\n 1.vehicle capacity {load} pieces\n 2.vehicle daily distance {dailyDistance} km\n 3.vehicle daily sites {dailySites}\n 4.depot opening hours {depotTime} min\n 5.vehicle shift times {vehicleTime} min\n 6.location time windows {tw}\n'
                    .format(load=self._data.vehicles.capacity[vehicle_id],
                            depotTime=self._data.timeWindows[self._data.depotIndex[vehicle_id]],
                            tw=route_timeWindow,
                            dailyDistance = self._data.vehicles.dailyDistanceLimit[vehicle_id],
                            dailySites = self._data.vehicles.nrJobLimit[vehicle_id],
                            vehicleTime = self._data.vehicles.vehicleTimeWindows[vehicle_id]
                            ) )
            plan_output += 'Result:\n 1.load of the route: {0} pcs\n'.format(route_load)
            plan_output += ' 2.distance of the route: {0} km\n'.format(route_dist)
            plan_output += ' 3.visited nr. sits: {0}\n'.format(route_siteCount)
            plan_output += ' 4.timespan of the route: ({0},{1}) min\n'.format(
                                                                                    route_startTime, route_endTime)
            plan_output += '   of which service time: {0} min\n'.format(route_serviceTime)
            print(plan_output)

        print('Total distance of all routes: {0} km\nTotal nr. visited sites: {1}\nTotal fulfilled demand: {2}\n'
              .format(total_dist,total_siteCount,total_fulfilledDemand))
        print('Dropped nodes: {0}\n').format(self.getDropped())
    
    def getDropped(self):
        dropped = []
        for idx in np.arange(self._routing.Size()):
            if self._assignment.Value(self._routing.NextVar(idx)) == idx:
                dropped.append(idx)
        return dropped


def main():
    # 使用默认的建模参数：
    model_parameters = pywrapcp.RoutingModel.DefaultModelParameters()
    routing = pywrapcp.RoutingModel(
            data.nrLocations, 
            data.nrVehicles, 
            data.depotIndex, 
            data.depotIndex,
            model_parameters)
    
    # 添加车辆成本:
    for n,v in enumerate(data.vehicles.costs):
        routing.SetFixedCostOfVehicle(v, n)
    
    # 添加距离成本：
    distEval = CreateDistanceEvaluator(data.locations)
    distance_evaluator = distEval.distance_evaluator # callback函数
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
    
    # 添加每辆车的最大行驶距离约束：
    add_capacity_constraints(routing, data.vehicles.dailyDistanceLimit,
                             distance_evaluator, 'dailyDistance')    
    
    # 添加每辆车的最大容量约束：
    demand_evaluator = CreateDemandEvaluator(data.demands).demand_evaluator
    add_capacity_constraints(routing, data.vehicles.capacity, 
                             demand_evaluator, 'capacity')
    
    # 添加每辆车的最多访问站点数约束：
    nrJobs_evaluator = CreateDemandEvaluator(
                                    data.visitedLocations).demand_evaluator
    add_capacity_constraints(routing, data.vehicles.nrJobLimit,
                             nrJobs_evaluator, 'dailyNrJobs')
    
    # 添加运营时间约束
    transitEval = CreateAllTransitEvaluators(data.vehicles, distEval.distances, 
                                             data.serviceTimes)
    add_transit_and_capacity_constraints(routing, data.latestWorkHours, 
                                     transitEval.evaluators, 
                                     int(np.max(data.latestWorkHours)), 'time')
    
    # 添加时间窗口约束
    add_timewindow_constraints(routing, data, 'time')
    
    # 设置搜索策略（本例中主要使用了默认参数，其他参数参考google教程）：
    # https://developers.google.com/optimization/routing/routing_options
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    
    # 设置惩罚项，允许有客户站点不被访问（当所有车辆达到约束上限时）
    # 否则问题有可能无解。
    non_depot = set(range(data.nrLocations))
    non_depot.difference_update(data.depotIndex)
    penalty = 400000
    nodes = [routing.AddDisjunction([c], penalty) for c in non_depot]
    
    # 求解
    assignment = routing.SolveWithParameters(search_parameters)
    
    # 打印结果
    printer = ConsolePrinter(data, routing, assignment, distEval.distances)
    printer.printAll()
    
    # plot routes
    dropped = []
    dropped = printer.getDropped()
    
    vehicle_routes = {}
    for veh in np.arange(data.nrVehicles):
        vehicle_routes[veh] = build_vehicle_route(routing, assignment, 
                                                          data.locations, veh)
    # Plotting of the routes in matplotlib.
    fig = plt.figure()
    ax = fig.add_subplot(111)
    # Plot all the nodes as black dots.
    clon, clat = zip(*[(c[0], c[1]) for i,c in enumerate(data.locations) if i not in dropped])
    ax.plot(clat, clon, 'k.')
    # plot the routes as arrows
    plot_vehicle_routes(vehicle_routes, ax, data)




