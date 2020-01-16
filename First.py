from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from itertools import repeat
from math import hypot


def distance(p1,p2):
    """Euclidean distance between two points."""
    x1,y1 = p1
    x2,y2 = p2
    return hypot(x2 - x1, y2 - y1)

dis_array=[]

pts = [(145,215), (151,264), (159,261), (130,254), (128,252), (163,247), (146,246), (161,242), (142,239), (163,236), (148,232), (128,231), (156,217), (129,214), (146,208), (164,208), (141,206), (147,193), (164,193), (129,189), (155,185), (139,182)]
pts1 = [(266,235), (295,272), (301,258), (309,260), (217,274), (218,278), (282,267), (242,249), (230,262), (249,268), (256,267), (265,257), (267,242), (259,265), (315,233), (329,252), (318,252), (329,224), (267,213), (275,192), (303,201), (208,217), (326,181)]

for x in range(len(pts)):
    dis = ([distance(*pair) for pair in zip(repeat(pts[0+x]),pts[0:])])
    dis_array.append(dis)
    x+=1


def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = dis_array

    data['demands'] = [0, 1100, 700, 800, 1400, 2100, 400, 800, 100, 500, 600, 1200, 1300, 1300, 300, 900, 2100, 1000, 900, 2500, 1800, 700]
    data['vehicle_capacities'] = [6000, 6000, 6000, 6000]
    data['num_vehicles'] = 4
    #data['demands'] = [0, 125, 84, 60, 500, 300, 175, 350, 150, 1100, 4100, 225, 300, 250, 500, 150, 100, 250, 120, 600, 500, 175, 75]
    #data['vehicle_capacities'] = [4500, 4500, 4500]
    #data['num_vehicles'] = 3
    data['depot'] = 0
    return data


def print_solution(data, manager, routing, assignment):
    """Prints assignment on console."""
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id+1)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index),
                                                 route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
    print('Total distance of all routes: {}m'.format(total_distance))
    print('Total load of all routes: {}'.format(total_load))


def main():
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)


    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if assignment:
        print_solution(data, manager, routing, assignment)


if __name__ == '__main__':
    main()

#dodać def get_routes