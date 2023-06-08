# gurobi version 10.0.0

import time
import gurobipy as gp
import math
from gurobipy import GRB
import networkx as nx
import numpy as np
from matplotlib import pyplot as plt


# function to create the graph
def create_graph(nodeList, arcList):
    # Create graph
    graph = nx.DiGraph()
    # add nodes
    graph.add_nodes_from(nodeList)
    # add arcs
    graph.add_weighted_edges_from(arcList)
    return graph

# create the MP of the MTRPD model
# graph -> Graph;
# numberOfDrones -> Number of available drones;
# numberOfTrips -> Number of possible trips;
# largeNumber -> large number;
# flightEndurance -> flight endurance of the drones;
# droneSpeed -> Drone Speed
# relax -> solving the relaxation of the problem?
def create_MTRPD_MP(graph: nx.DiGraph,
                    numberOfDrones: int,
                    numberOfTrips: int,
                    flightEndurance: float,
                    droneSpeed: int,
                    relax: bool):
    # Initialize the model
    model = gp.Model()

    # number of Levels / customers to be visited in the backward order
    numberOfLevels = graph.number_of_nodes() - 1
    if numberOfLevels<2:
        return "No need to build a model. We have only one customer"

    # get the traveling times (dict) --> (arc) : weight
    travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
    travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}
    # nodes - numberOfLevels dict -> (node, level): Gurobi variables
    nodesVisitedByTruck = {}

    # truck arcs dict -> (arc, level) : Gurobi varialbes
    arcsVisitedByTruck = {}
    # drone arcs dict -> (arc, drone, level) : Gurobi varialbes
    arcsVisitedByDrone = {}

    # Arriving time of Truck in the level l
    # truck customers arrivals dict -> level : Gurobi variable
    truckArrivingTime = {}

    # Arriving time of a drone at a customer j in the level l in its qth trip
    # drone customers arrivals relaxation dict
    droneArrivingTime1 = {} # end Node - level : Gurobi variable
    droneArrivingTime2 = {} # start Node - drone - drone trip : Gurobi variable
    droneArrivingTime3 = {} # end Node - level : Gurobi variable
    lambda1Vars = {} # end Node - level : Gurobi variable
    lambda2Vars = {} # end Node - level : Gurobi variable
    lambda3Vars = {} # end Node - level : Gurobi variable
    lambda4Vars = {} # end Node - level : Gurobi variable

    objectiveValue = {'eta':model.addVar(vtype=GRB.CONTINUOUS,lb= 0, name = 'eta')}
    # Gurobi Variables:
    # binary node variable to assign a customer to the truck at level l.
    # x_i^l == 1 if node i outOf T is visited in the backward position l
    for node in graph.nodes():
        for level in range(numberOfLevels, 0, -1):
            nodesVisitedByTruck[(node, level)] = \
                model.addVar(
                    vtype=GRB.BINARY,
                    lb=0,
                    ub=1,
                    name="x_{0}^{1}".format(node, level)
                )
    for arc in graph.edges():
        # In the first level (numberOfLevels) we have only the arcs from the depot to all other customers
        if arc[0] == 0:
            for level in range(numberOfLevels, 0, -1):
                # binary arc variable on the truck delivery route.
                # y_ij^l == 1 if e_(ij) is being used by the truck and exactly l customers need to be served by the truck after node i
                arcsVisitedByTruck[(arc[0], arc[1], level)] = \
                    model.addVar(
                        vtype=GRB.BINARY,
                        lb=0,
                        ub=1,
                        name="y_{0}_{1}^{2}".format(arc[0], arc[1], level)
                    )

        else:
            for level in range(numberOfLevels - 1, 0, -1):
                # binary arc variable on the truck delivery route.
                # y_ij^l == 1 if e_(ij) is being used by the truck and exactly l customers need to be served by the truck after node i
                arcsVisitedByTruck[(arc[0], arc[1], level)] = \
                    model.addVar(
                        vtype=GRB.BINARY,
                        lb=0,
                        ub=1,
                        name="y_{0}_{1}^{2}".format(arc[0], arc[1], level)
                    )

    # binary arc variable on the drone delivery route.
    # f_ijkt^l == 1 if the truck launches drone k to serve node j outOf D in its tth trip, while waiting at node i in level l
    for arc in graph.edges():  # arc[0] != arc[1]
        if arc[1] != 0:
            for level in range(numberOfLevels, 0, -1):
                for drone in range(1, numberOfDrones + 1, 1):
                        arcsVisitedByDrone[(arc[0], arc[1], drone, level)] = \
                            model.addVar(
                                vtype=GRB.BINARY,
                                lb=0,
                                ub=1,
                                name="z_{0}_{1}_{2}^{3}".format(arc[0], arc[1], drone, level)
                            )
    for level in range(numberOfLevels, 0, -1):
        # non-negative continuous variable representing the arrivel time at the lth last customer served by the truck
        # t^l
        truckArrivingTime[level] = model.addVar(
            vtype=GRB.CONTINUOUS,
            lb=0,
            name="t^{0}".format(level)
        )
        # non-negative continuous variable representing the arrivel time at node j outOf D served by a drone in its tth trip
        # while truck waiting at the lth last customer
        for endNode in graph.nodes():
            if endNode != 0:
                droneArrivingTime1[(endNode, level)] = model.addVar(
                    vtype=GRB.CONTINUOUS,
                    lb=0,
                    name="pi1_{0}^{1}".format(endNode, level)
                )
                droneArrivingTime3[(endNode, level)] = model.addVar(
                    vtype=GRB.CONTINUOUS,
                    lb=0,
                    name="pi3_{0}^{1}".format(endNode, level)
                )
                lambda1Vars[(endNode, level)] = model.addVar(
                    vtype=GRB.CONTINUOUS,
                    lb=0,
                    name="lambda1_{0}^{1}".format(endNode, level))
                lambda2Vars[(endNode, level)] = model.addVar(
                    vtype=GRB.CONTINUOUS,
                    lb=0,
                    name="lambda2_{0}^{1}".format(endNode, level))
                lambda3Vars[(endNode, level)] = model.addVar(
                    vtype=GRB.CONTINUOUS,
                    lb=0,
                    name="lambda3_{0}^{1}".format(endNode, level))
                lambda4Vars[(endNode, level)] = model.addVar(
                    vtype=GRB.CONTINUOUS,
                    lb=0,
                    name="lambda4_{0}^{1}".format(endNode, level))
    for launchNode in graph.nodes():
        for drone in range(1, numberOfDrones+1, 1):
            for droneTrip in range(1, numberOfTrips+1, 1):
                droneArrivingTime2[(launchNode, drone, droneTrip)] = model.addVar(
                    vtype=GRB.CONTINUOUS,
                    lb=0,
                    name="pi2_{0}_{1}_{2}".format(launchNode, drone, droneTrip)
                )


    # truck constraints:
    constraints = {i: [] for i in range(1, 14)}
    #(1) the objective
    constraints[1].append(model.addConstr(
        objectiveValue['eta']
        >=
        sum(
            truckArrivingTime[level]
            for level in range(numberOfLevels, 0, -1)
        )
        +
        # drones
        sum(
            droneArrivingTime1[(endNode, level)]
            for endNode in graph.nodes() if endNode != 0
            for level in range(numberOfLevels, 0, -1)
        )
        +
        sum(
            droneArrivingTime2[(launchNode, drone, droneTrip)]
            for launchNode in graph.nodes()
            for drone in range(1, numberOfDrones+1, 1)
            for droneTrip in range(1, numberOfTrips+1, 1)
        )
        +
        sum(
            droneArrivingTime3[(endNode, level)]
            for endNode in graph.nodes() if endNode != 0
            for level in range(numberOfLevels, 0, -1)
        ),
        name= "(1)"
    ))
    # (2) the truck ends the route at a customer in the level 1 (the last level)
    # Definition: startNode is the starting node of an arc
    constraints[2].append(model.addConstr(
        sum(
            nodesVisitedByTruck[(startNode, 1)]
            for startNode in range(1, len(graph.nodes))
        )
        == 1,
        name='(2a)'
    ))
    # the depot node shouldn't be visited by the truck at level 1
    model.addConstr(
        nodesVisitedByTruck[(0, 1)] == 0,
        name= '(2b)'
    )
    # (3) The truck leaves the depot to a node j in one of the levels l
    # Definition: endNode is the ending node of an arc
    constraints[3].append(model.addConstr(
        sum(
            arcsVisitedByTruck[(0, endNode, level)]
            for endNode in graph.nodes() if endNode != 0
            for level in range(numberOfLevels, 0, -1)
        )
        == 1,
        name='(3)'
    ))
    # (4) The connection between the nodes and arcs visited by the truck. - outbound
    # i.e. if a node i is visited by the truck in the level l+1 , then the truck needs to use one of the arcs in the level l to leave this node i
    for startNode in graph.nodes():
        for level in range(numberOfLevels - 1, 0, -1):
            constraints[4].append(model.addConstr(
                sum(
                    arcsVisitedByTruck[(startNode, endNode, level)]
                    for endNode in graph.nodes() if endNode != 0 and endNode != startNode
                )
                == nodesVisitedByTruck[(startNode, level + 1)],
                name='(4_[i = {0}, l = {1}])'.format(startNode, level)
            ))
    # (5) The connection between the nodes and arcs visited by the truck. - inbound
    for endNode in graph.nodes():
        if endNode != 0:
            for level in range(numberOfLevels - 1, 0, -1):
                constraints[5].append(model.addConstr(arcsVisitedByTruck[(0, endNode, level)] +
                                                      sum(
                                                          arcsVisitedByTruck[(startNode, endNode, level)]
                                                          for startNode in graph.nodes() if
                                                          startNode != 0 and startNode != endNode
                                                      )
                                                      == nodesVisitedByTruck[(endNode, level)],
                                                      name='(5_[j = {0}, l = {1}])'.format(endNode, level)
                                                      ))
    # (6) The connection between the node at the last level N and the arcs out of the depot to the nodes in this level
    for endNode in graph.nodes():
        if endNode != 0:
            constraints[6].append(model.addConstr(
                arcsVisitedByTruck[(0, endNode, numberOfLevels)] == nodesVisitedByTruck[(endNode, numberOfLevels)],
                name='(6_[j = {0}])'.format(endNode)
            ))
    # (7) all nodes/customers are visited either by the truck or the drone
    for endNode in graph.nodes():
        if endNode != 0:
            constraints[7].append(model.addConstr(
                sum(
                    nodesVisitedByTruck[(endNode, level)]
                    for level in range(numberOfLevels, 0, -1)
                )
                +
                sum(
                    arcsVisitedByDrone[(startNode, endNode, drone, level)]
                    for startNode in graph.nodes() if startNode != endNode and startNode != 0
                    for drone in range(1, numberOfDrones + 1, 1)
                    for level in range(numberOfLevels - 1, 0, -1)
                )
                +
                sum(
                    arcsVisitedByDrone[(0, endNode, drone, level)]
                    for drone in range(1, numberOfDrones + 1, 1)
                    for level in range(numberOfLevels - 1, 0, -1)
                )
                == 1,
                name='(7_[i = {0}])'.format(endNode)
            ))
    # (8) drone trips constraints
    for level in range(numberOfLevels, 0, -1):
        for startNode in graph.nodes():
            for drone in range(1, numberOfDrones + 1, 1):
                constraints[8].append(model.addConstr(
                    sum(
                        arcsVisitedByDrone[(startNode, endNode, drone, level)]
                        for endNode in graph.nodes if endNode != startNode and endNode != 0
                    )
                    <= numberOfTrips * nodesVisitedByTruck[(startNode, level)],
                    name='(8_[i = {0}, k = {1}, l = {2}])'.format(startNode, drone, level)
                ))
    # (9) Arriving time of the truck in the level l
    for level in range(numberOfLevels, 0, -1):
        for drone in range(1, numberOfDrones + 1, 1):
            # first step of the truck
            if level == numberOfLevels:
                constraints[9].append(model.addConstr(
                    # t^l
                    truckArrivingTime[level]
                    >=
                    sum(
                        travelingTimeTruck[(0, endNode)] * arcsVisitedByTruck[(0, endNode, level)]
                        for endNode in graph.nodes() if endNode != 0
                    ),
                    name='(9_[l = {0}])'.format(level)
                ))
            # starting from the second last step
            else:
                constraints[9].append(model.addConstr(
                    # t^l
                    truckArrivingTime[level]
                    >=
                    # previous traveling (and waiting time)
                    # we separate the following two sums because in level numberOfLevels there are only arcs out of the depot
                    # truck arcs out of the depot 0
                    sum(
                        travelingTimeTruck[(0, endNode)] * arcsVisitedByTruck[(0, endNode, previousLevel)]
                        for endNode in graph.nodes() if endNode != 0
                        for previousLevel in range(numberOfLevels, level, -1)
                    )
                    +
                    # arcs out of nodes other than the depot 0
                    sum(
                        travelingTimeTruck[(startNode, endNode)] * arcsVisitedByTruck[(startNode, endNode, previousLevel)]
                        for startNode in graph.nodes() if startNode != 0
                        for endNode in graph.nodes() if endNode != startNode and endNode != 0
                        for previousLevel in range(numberOfLevels - 1, level, -1)
                        # since out of level numberOfLevels there are only arcs out of the depot
                    )
                    +
                    #previous drone trips
                    sum(
                        (travelingTimeDrone[(startNode, endNode)] + travelingTimeDrone[(endNode, startNode)]) *
                        arcsVisitedByDrone[(startNode, endNode, drone, previousLevel)]
                        for startNode in graph.nodes()  # if startNode != 0
                        for endNode in graph.nodes() if endNode != startNode and endNode != 0
                        for previousLevel in range(numberOfLevels, level, -1)
                        # since out of level numberOfLevels there are only arcs out of the depot
                    )
                    # current traveling time of the truck
                    +
                    sum(
                        travelingTimeTruck[(startNode, endNode)] * arcsVisitedByTruck[(startNode, endNode, level)]
                        for startNode in graph.nodes()
                        for endNode in graph.nodes() if endNode != startNode and endNode != 0
                    ),
                    name='(9_[l = {0}])'.format(level)
                ))

    # (10) Arriving time of a drone in its qth trip at customer j in the level l
    for endNode in graph.nodes():
        if endNode != 0:
            for level in range(numberOfLevels, 0, -1):
                # we visited all the customers before by the truck using the longest edge
                worstCaseTruckArrivingTime = (len(graph.nodes())-1) *max(travelingTimeTruck.values())
                constraints[10].append(model.addConstr(
                    droneArrivingTime1[(endNode, level)]
                    ==
                    lambda4Vars[(endNode, level)]*worstCaseTruckArrivingTime,
                name='(10a_[j = {0}, l = {1}])'.format(endNode,level)
                ))
                constraints[10].append(model.addConstr(
                    truckArrivingTime[level]
                    ==
                    (lambda3Vars[(endNode, level)] + lambda4Vars[(endNode, level)]) * worstCaseTruckArrivingTime,
                name='(10b_[j = {0}, l = {1}])'.format(endNode,level)
                ))
                constraints[10].append(model.addConstr(
                    sum(arcsVisitedByDrone[(launchNode, endNode, drone, level)]
                        for launchNode in graph.nodes() if launchNode != endNode
                        for drone in range(1, numberOfDrones + 1, 1)
                        )
                    ==
                    (lambda2Vars[(endNode, level)] + lambda4Vars[(endNode, level)]),
                name='(10c_[j = {0}, l = {1}])'.format(endNode,level)
                ))
                constraints[10].append(model.addConstr(
                    (lambda1Vars[(endNode, level)] + lambda2Vars[(endNode, level)] + lambda3Vars[(endNode, level)] + lambda4Vars[(endNode, level)])
                    ==
                    1,
                name='(10d_[j = {0}, l = {1}])'.format(endNode,level)
                ))
    # (11) Arriving time of a drone in its qth trip at customer j in the level l
    for launchNode in graph.nodes():
        for drone in range(1, numberOfDrones+1, 1):
            for droneTrip in range(1, numberOfTrips + 1, 1):
                tdMin = min([2*travelingTimeDrone[(launchNode,endNode)] for endNode in graph.nodes() if endNode != 0 and endNode != launchNode])
                constraints[11].append(model.addConstr(
                    droneArrivingTime2[(launchNode,drone, droneTrip)]
                    >=
                    (sum(arcsVisitedByDrone[(launchNode, endNode, drone, level)]
                                                   for endNode in graph.nodes() if endNode != 0 and endNode != launchNode
                                                   for level in range(numberOfLevels, 0, -1)) - droneTrip) * tdMin ,
                    name='(11_[i = {0}, k = {1}, q = {2}])'.format(launchNode, drone, droneTrip)
                ))
    # (12) Arriving time of a drone in its qth trip at customer j in the level l
    for endNode in graph.nodes():
        if endNode != 0:
            for level in range(numberOfLevels, 0, -1):
                constraints[12].append(model.addConstr(
                    droneArrivingTime3[(endNode, level)]
                    ==
                    sum(travelingTimeDrone[(launchNode, endNode)] * arcsVisitedByDrone[(launchNode, endNode, drone, level)]
                                                   for launchNode in graph.nodes() if launchNode != endNode
                                                   for drone in range(1, numberOfDrones+1, 1)),
                    name='(12_[j = {0}, l = {1}])'.format(endNode, level)
                ))
    #(13) the flight endurance constraint of each drone
    for launchNode in graph.nodes():
        for endNode in graph.nodes():
            if endNode != launchNode and endNode != 0:
                for drone in range(1, numberOfDrones+1, 1):
                    for droneTrip in range(1, numberOfTrips+1, 1):
                        for level in range(numberOfLevels, 0, -1):
                            constraints[13].append(model.addConstr(
                                (travelingTimeDrone[(launchNode, endNode)]+travelingTimeDrone[(endNode, launchNode)])
                                * arcsVisitedByDrone[(launchNode, endNode, drone, level)]
                                <=
                                flightEndurance,
                                name='(13_[i = {0}, j = {1}, k = {2}, l = {3}])'.format(launchNode, endNode, drone, level)
                            ))


    # (0) set the objective: Sum of all traveling times of the truck and the drones
    model.setObjective(
        objectiveValue['eta']
        , GRB.MINIMIZE,
    )

    droneArrivingTime = [droneArrivingTime1, droneArrivingTime2, droneArrivingTime3]

    # uncomment the following command to export the model to a text file
    # model.write(
    #     "MTRPD_LBBD_MP_[Instance = {0}, Drones = {1}, Allowed Nr. of Drone Trips = {2}, Flight Range = {3}].lp".format(
    #         graph, numberOfDrones, numberOfTrips, flightEndurance))
    # return ralaxed model
    if relax == True:
        for key in nodesVisitedByTruck.keys():
            nodesVisitedByTruck[key].vtype = GRB.CONTINUOUS
        for key in arcsVisitedByTruck.keys():
            arcsVisitedByTruck[key].vtype = GRB.CONTINUOUS
        for key in arcsVisitedByDrone.keys():
            arcsVisitedByDrone[key].vtype = GRB.CONTINUOUS
        return model, constraints, objectiveValue, nodesVisitedByTruck,  arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime
    else:
        return model, constraints, objectiveValue, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime

# create the SP of the MTRPD model
# graph -> Graph;
# numberOfDrones -> Number of available drones;
# numberOfTrips -> Number of possible trips;
# largeNumber -> large number;
# flightEndurance -> flight endurance of the drones;
# droneSpeed -> Drone Speed
# chosenNodesTruck -> the nodes visited by the truck according to the MP solution
# chosenArcsTruck -> the arcs connecting the chosenNodesTruck
# relax -> solving the relaxation of the problem?
def create_MTRPD_SP(graph: nx.DiGraph,
                    numberOfDrones: int,
                    numberOfTrips: int,
                    largeNumber: int,
                    flightEndurance: float,
                    droneSpeed: int,
                    chosenNodesTruck: dict,
                    chosenArcsTruck: dict,
                    relax: bool):

    # Initialize the model
    model = gp.Model()

    # number of Levels / customers to be visited in the backward order
    numberOfLevels = graph.number_of_nodes() - 1

    # get the traveling times (dict) --> (arc) : weight
    travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
    travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}

    # arcs dict -> arc - level - drone - trip : Gurobi varialbes
    arcsVisitedByDrone = {}

    # Arriving time of Truck in the level l
    # truck customers arrivals dict -> level : Gurobi variable
    truckArrivingTime = {}

    # Arriving time of a drone at a customer j in the level l in its qth trip
    # drone customers arrivals dict -> end Node - drone trip - level : Gurobi variable
    droneArrivingTime = {}

    # Gurobi Variables:
    # binary arc variable on the drone delivery route.
    # f_ijkt^l == 1 if the truck launches drone k to serve node j outOf D in its tth trip, while waiting at node i in level l
    for arc in graph.edges():  # arc[0] != arc[1]
        if arc[1] != 0:
            for level in range(numberOfLevels, 0, -1):
                for drone in range(1, numberOfDrones + 1, 1):
                    for droneTrip in range(1, numberOfTrips + 1, 1):
                        arcsVisitedByDrone[(arc[0], arc[1], drone, droneTrip, level)] = \
                            model.addVar(
                                vtype=GRB.BINARY,
                                lb=0,
                                ub=1,
                                name="f_{0}_{1}_{2}_{3}^{4}".format(arc[0], arc[1], drone, droneTrip, level)
                            )
    for level in range(numberOfLevels, 0, -1):
        # non-negative continuous variable representing the arrivel time at the lth last customer served by the truck
        # t^l
        truckArrivingTime[level] = model.addVar(
            vtype=GRB.CONTINUOUS,
            lb=0,
            name="t^{0}".format(level)
        )
        # non-negative continuous variable representing the arrivel time at node j outOf D served by a drone in its tth trip
        # while truck waiting at the lth last customer
        for node in graph.nodes():
            if node != 0:
                for droneTrip in range(1, numberOfTrips + 1, 1):
                    droneArrivingTime[(node, droneTrip, level)] = model.addVar(
                        vtype=GRB.CONTINUOUS,
                        lb=0,
                        name="pi_{0}_{1}^{2}".format(node, droneTrip, level)
                    )

    # truck constraints:
    constraints = {i: [] for i in range(19, 26)}

    # (19) all nodes/customers are visited either by the truck or the drone
    for endNode in graph.nodes():
        if endNode != 0:
            constraints[19].append(model.addConstr(
                sum(
                    chosenNodesTruck[(endNode, level)]
                    for level in range(numberOfLevels, 0, -1)
                )
                +
                sum(
                    arcsVisitedByDrone[(startNode, endNode, drone, droneTrip, level)]
                    for startNode in graph.nodes() if startNode != endNode and startNode != 0
                    for drone in range(1, numberOfDrones + 1, 1)
                    for droneTrip in range(1, numberOfTrips + 1, 1)
                    for level in range(numberOfLevels, 0, -1)
                )
                +
                sum(
                    arcsVisitedByDrone[(0, endNode, drone, droneTrip, level)]
                    for drone in range(1, numberOfDrones + 1, 1)
                    for droneTrip in range(1, numberOfTrips + 1, 1)
                    for level in range(numberOfLevels, 0, -1)
                )
                == 1,
                name='(19_[i = {0}])'.format(endNode)
            ))
    # (20) drone trips constraints
    for level in range(numberOfLevels, 0, -1):
        for startNode in graph.nodes():
            for drone in range(1, numberOfDrones + 1, 1):
                constraints[20].append(model.addConstr(
                    sum(
                        arcsVisitedByDrone[(startNode, endNode, drone, droneTrip, level)]
                        for endNode in graph.nodes if endNode != startNode and endNode != 0
                        for droneTrip in range(1, numberOfTrips + 1, 1)
                    )
                    <= numberOfTrips * chosenNodesTruck[(startNode, level)],
                    name='(20_[i = {0}, k = {1}, l = {2}])'.format(startNode, drone, level)
                ))
    # (21) Arriving time of the truck in the level l
    for level in range(numberOfLevels, 0, -1):
        for drone in range(1, numberOfDrones + 1, 1):
            # first step of the truck
            if level == numberOfLevels:
                constraints[21].append(model.addConstr(
                    # t^l
                    truckArrivingTime[level]
                    >=
                    sum(
                        travelingTimeTruck[(0, endNode)] * chosenArcsTruck[(0, endNode, level)]
                        for endNode in graph.nodes() if endNode != 0
                    ),
                    name='(21_[l = {0}])'.format(level)
                ))
            # starting from the second last step
            else:
                constraints[21].append(model.addConstr(
                    # t^l
                    truckArrivingTime[level]
                    >=
                    # previous traveling (and waiting time)
                    # we separate the following two sums because in level numberOfLevels there are only arcs out of the depot
                    # truck arcs out of the depot 0
                    sum(
                        travelingTimeTruck[(0, endNode)] * chosenArcsTruck[(0, endNode, previousLevel)]
                        for endNode in graph.nodes() if endNode != 0
                        for previousLevel in range(numberOfLevels, level, -1)
                    )
                    +
                    # arcs out of nodes other than the depot 0
                    sum(
                        travelingTimeTruck[(startNode, endNode)] * chosenArcsTruck[
                            (startNode, endNode, previousLevel)]
                        for startNode in graph.nodes() if startNode != 0
                        for endNode in graph.nodes() if endNode != startNode and endNode != 0
                        for previousLevel in range(numberOfLevels - 1, level, -1)
                        # since out of level numberOfLevels there are only arcs out of the depot
                    )
                    +
                    sum(
                        (travelingTimeDrone[(startNode, endNode)] + travelingTimeDrone[(endNode, startNode)]) *
                        arcsVisitedByDrone[(startNode, endNode, drone, droneTrip, previousLevel)]
                        for startNode in graph.nodes()  # if startNode != 0
                        for endNode in graph.nodes() if endNode != startNode and endNode != 0
                        for droneTrip in range(1, numberOfTrips + 1, 1)
                        for previousLevel in range(numberOfLevels, level, -1)
                        # since out of level numberOfLevels there are only arcs out of the depot
                    )
                    # current traveling time of the truck
                    +
                    sum(
                        travelingTimeTruck[(startNode, endNode)] * chosenArcsTruck[(startNode, endNode, level)]
                        for startNode in graph.nodes()
                        for endNode in graph.nodes() if endNode != startNode and endNode != 0
                    ),
                    name='(21_[l = {0}])'.format(level)
                ))


    # (22) Arriving time of a drone in its qth trip at customer j in the level l
    for endNode in graph.nodes():
        if endNode != 0:
            for drone in range(1, numberOfDrones + 1, 1):
                for droneTrip in range(1, numberOfTrips + 1, 1):
                    for level in range(numberOfLevels, 0, -1):
                        if droneTrip == 1:
                            constraints[22].append(model.addConstr(
                                # pi_j_q^l
                                droneArrivingTime[(endNode, droneTrip, level)]
                                >=
                                truckArrivingTime[level]
                                +
                                # current traveling time to a customer j by the drone
                                sum(
                                    travelingTimeDrone[(launchNode, endNode)] * arcsVisitedByDrone[
                                        (launchNode, endNode, drone, droneTrip, level)]
                                    for launchNode in graph.nodes() if launchNode != endNode
                                )
                                - largeNumber * (1 - sum(
                                    arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                                    for launchNode in graph.nodes() if launchNode != endNode
                                )),
                                name='(22_[j = {0}, k = {1}, q = 1, l = {2}])'.format(endNode, drone, level)
                            ))
                        # q > 1
                        else:
                            constraints[22].append(model.addConstr(
                                # pi_j_q^l
                                droneArrivingTime[(endNode, droneTrip, level)]
                                >=
                                truckArrivingTime[level]
                                +
                                # previous trips traveling time for a drone k
                                sum(
                                    (travelingTimeDrone[(launchNode, previousEndNode)] + travelingTimeDrone[
                                        (previousEndNode, launchNode)])
                                    * arcsVisitedByDrone[(launchNode, previousEndNode, drone, previousDroneTrip, level)]
                                    for launchNode in graph.nodes() if launchNode != endNode
                                    for previousEndNode in graph.nodes()
                                    if
                                    previousEndNode != 0 and previousEndNode != endNode and previousEndNode != launchNode
                                    for previousDroneTrip in range(1, droneTrip, 1)
                                )
                                +
                                # current traveling time for a drone k to a customer j
                                sum(
                                    travelingTimeDrone[(launchNode, endNode)] * arcsVisitedByDrone[
                                        (launchNode, endNode, drone, droneTrip, level)]
                                    for launchNode in graph.nodes() if launchNode != endNode
                                )
                                - largeNumber * (1 - sum(
                                    arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                                    for launchNode in graph.nodes() if launchNode != endNode
                                )),
                                name='(22_[j = {0}, k = {1}, q = {2}, l = {3}])'.format(endNode, drone, droneTrip,
                                                                                          level)
                            ))

    # (23) Each drone is visiting only one node per trip before coming back to the truck
    for launchNode in graph.nodes():
        for drone in range(1, numberOfDrones + 1, 1):
            for droneTrip in range(1, numberOfTrips + 1, 1):
                constraints[23].append(model.addConstr(
                    sum(
                        arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                        for endNode in graph.nodes() if endNode != launchNode and endNode != 0
                        for level in range(numberOfLevels, 0, -1)
                    )
                    <= 1,
                    name='(23_[i = {0}, k = {1}, q = {2}])'.format(launchNode, drone, droneTrip)
                ))

    # (24) trip order is ensured
    if numberOfTrips >= 2:
        for launchNode in graph.nodes():
            for drone in range(1, numberOfDrones + 1, 1):
                for droneTrip in range(1, numberOfTrips, 1):
                    for level in range(numberOfLevels, 0, -1):
                        constraints[24].append(model.addConstr(
                            sum(
                                arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                                for endNode in graph.nodes() if endNode != launchNode and endNode != 0
                            )
                            >=
                            sum(
                                arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip + 1, level)]
                                for endNode in graph.nodes() if endNode != launchNode and endNode != 0
                            ),
                            name='(24_[i = {0}, k = {1}, q = {2}, l = {3}])'.format(launchNode, drone, droneTrip,
                                                                                      level)
                        ))

    # (25) the flight endurance constraint of each drone
    for launchNode in graph.nodes():
        for endNode in graph.nodes():
            if endNode != launchNode and endNode != 0:
                for drone in range(1, numberOfDrones + 1, 1):
                    for droneTrip in range(1, numberOfTrips + 1, 1):
                        for level in range(numberOfLevels, 0,
                                           -1):  # level numberOfLevels is exculed since it's not considered. It's not possible in this model to start a drone in Level numberOfLevels, since if we send one drone, we would be in level numberOfLevels-1
                            constraints[25].append(model.addConstr(
                                (travelingTimeDrone[(launchNode, endNode)] + travelingTimeDrone[(endNode, launchNode)])
                                * arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                                <=
                                flightEndurance,
                                name='(25_[i = {0}, j = {1}, k = {2}, q = {3}, l = {4}])'.format(launchNode, endNode,
                                                                                                   drone, droneTrip,
                                                                                                   level)
                            ))

    # (0) set the objective: Sum of all traveling times of the truck and the drones
    model.setObjective(
        # truck
        sum(
            truckArrivingTime[level]
            for level in range(numberOfLevels, 0, -1)
        )
        +
        # drones
        sum(
            droneArrivingTime[(endNode, q, level)]
            for endNode in graph.nodes() if endNode != 0
            for q in range(1, numberOfTrips + 1, 1)
            for level in range(numberOfLevels, 0, -1)
        )
        , GRB.MINIMIZE,
    )

    # uncomment the following command to export the model to a text file
    # model.write(
    #     "MTRPD_LBBD_SP_[Instance = {0}, Drones = {1}, Allowed Nr. of Drone Trips = {2}, Flight Range = {3}].lp".format(
    #         graph, numberOfDrones, numberOfTrips, flightEndurance))
    # return ralaxed model
    if relax == True:
        return model.relax(), constraints, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime

    return model, constraints, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime


# plot the solution graph
def plot_graph(G, dictNodePos, *args):
    if len(args) != 0:
        nx.draw(G, with_labels=True, pos=dictNodePos, node_color=[color for color in args[0].values()])
    else:
        nx.draw(G, with_labels=True, pos=dictNodePos)
    arcLabels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos=dictNodePos, edge_labels=arcLabels)
    # plt.show() needs to be left out to save the figure
    plt.show()


# return the list of arcs of the form (i, j, distance(i,j))
def read_instance_file(fileName):
    with open(fileName, 'r') as instance:
        # linesNumbers = [8, 10] #line 8 is the depot and starting from line 10 we have the customer nodes
        nodeNumber = 1
        graphLocations = {}
        for i, line in enumerate(instance):
            if line == '':
                break
            if i == 7:
                lineList = line.split(' ')
                graphLocations[0] = (float(lineList[0]), float(lineList[1]))
            if i > 8:
                lineList = line.split(' ')
                graphLocations[nodeNumber] = (float(lineList[0]), float(lineList[1]))
                nodeNumber += 1
        instance.close()

    graphArcs = [(i, j)
                 for i in graphLocations.keys()
                 for j in graphLocations.keys() if j != i
                 ]

    graphArcsWithWeights = []
    for arc in graphArcs:
        # euclidean distance
        distance = math.ceil(np.sqrt((graphLocations[arc[0]][0] - graphLocations[arc[1]][0]) ** 2 + (
                    graphLocations[arc[0]][1] - graphLocations[arc[1]][1]) ** 2))
        graphArcsWithWeights.append((arc[0], arc[1], distance))

    graph = create_graph(graphLocations.keys(), graphArcsWithWeights)

    return graphLocations, graphArcsWithWeights, graph

# function to solve the MP model
def solve_MP_model(model: gp.Model,
                nodesVisitedByTruck: dict,
                arcsVisitedByTruck: dict,
                arcsVisitedByDrone: dict,
                truckArrivingTime: dict,
                droneArrivingTime: dict,
                efficientSolvingGurobi: bool,
                graph: nx.DiGraph,
                droneSpeed: int,
                t1: time.time):
    # traveling time of truck and drone
    # get the traveling times (dict) --> (arc) : weight
    travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
    travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}

    # deactivate any automatic cuts, heuristics and presolve Gurobi techniques
    if efficientSolvingGurobi == False:
        model.setParam('Cuts', 0)
        model.setParam('ImpliedCuts', 0)
        model.setParam('CliqueCuts', 0)
        model.setParam('Heuristics', 0)
        model.setParam('Presolve', 0)
    # set Time limit
    model.setParam('TimeLimit', 60*60)

    def cb(model, where):
        if where == GRB.Callback.MIP:
            if time.time() - t1 > 3600:
                model.terminate()

    # solve the model
    model.optimize(callback = cb)


    if model.status == GRB.INFEASIBLE:
        return False
    if model.status == GRB.INTERRUPTED:
        result = {}
        return result

    # solution dicts of the model
    chosenNodesVisitedByTruck = {}  # node: 1
    chosenArcsVisitedByTruck = {}   # (startNode, endNode, level): 1
    chosenArcsVisitedByDrone = {}   # (startNode, endNode, drone, level): 1
    resultTruckArrivingTime = {}    # level: value > 0
    resultDroneArrivingTime1 = {}   # (endNode,level): value > 0
    resultDroneArrivingTime2 = {}   # (launchNode, drone, droneTrip): value > 0
    resultDroneArrivingTime3 = {}   # (endNode,level): value > 0
    resultDroneArrivingTime = [resultDroneArrivingTime1, resultDroneArrivingTime2, resultDroneArrivingTime3]


    # solution graph
    H = nx.DiGraph()
    # define the color of the nodes. red = truck node. green = drone node.
    colors = {}
    # the depot
    H.add_node(0)
    colors[0] = 'red'
    # add the truck nodes to the solution graph
    for (node, level) in nodesVisitedByTruck.keys():
        if nodesVisitedByTruck[(node, level)].X > 0.99:
            chosenNodesVisitedByTruck[(node, level)] = 1
            H.add_node(node)
            colors[node] = 'red'
        else:
            chosenNodesVisitedByTruck[(node, level)] = 0
    # add the drone nodes to the subdigraph
    for node in graph.nodes():
        if node not in H:
            H.add_node(node)
            colors[node] = 'green'

    # add the truck edges to the solution graph
    for (startNode, endNode, level) in arcsVisitedByTruck.keys():
        if arcsVisitedByTruck[(startNode, endNode, level)].X > 0.99:
            chosenArcsVisitedByTruck[(startNode, endNode, level)] = 1
            H.add_edge(startNode, endNode, weight=travelingTimeTruck[(startNode, endNode)])
        else:
            chosenArcsVisitedByTruck[(startNode, endNode, level)] = 0
    # add the drone edges to the solution graph
    for (startNode, endNode, drone, level) in arcsVisitedByDrone.keys():
        if arcsVisitedByDrone[(startNode, endNode, drone, level)].X > 0.99:
            chosenArcsVisitedByDrone[(startNode, endNode, drone, level)] = 1
            H.add_edge(startNode, endNode, weight=travelingTimeDrone[(startNode, endNode)])
            H.add_edge(endNode, startNode, weight=travelingTimeDrone[(startNode, endNode)])
        else:
            chosenArcsVisitedByDrone[(startNode, endNode, drone, level)] = 0
    # get the truck arriving time in each level
    for l in truckArrivingTime.keys():
        if truckArrivingTime[l].X > 0:
            resultTruckArrivingTime[l] = truckArrivingTime[l].X
    # get the drone arriving time in each level in each droneTrip
    for (endNode,level) in droneArrivingTime[0].keys():
        if droneArrivingTime[0][(endNode,level)].X > 0:
            resultDroneArrivingTime1[(endNode,level)] = droneArrivingTime[0][(endNode, level)].X
    for (launchNode, drone, droneTrip) in droneArrivingTime[1].keys():
        if droneArrivingTime[1][(launchNode, drone, droneTrip)].X > 0:
            resultDroneArrivingTime2[(launchNode, drone, droneTrip)] = droneArrivingTime[1][(launchNode, drone, droneTrip)].X
    for (endNode,level) in droneArrivingTime[2].keys():
        if droneArrivingTime[2][(endNode,level)].X > 0:
            resultDroneArrivingTime3[(endNode,level)] = droneArrivingTime[2][(endNode, level)].X

    result = {}
    result['nodesColors'] = colors #red = Truck, green = Drone
    result['truckNodes'] = chosenNodesVisitedByTruck
    result['truckArcs'] = chosenArcsVisitedByTruck
    result['droneArcs'] = chosenArcsVisitedByDrone
    result['truckArrivingTimes'] = resultTruckArrivingTime
    result['droneArrivingTimes'] = resultDroneArrivingTime
    result['objectiveValue' ] = model.objVal
    return result

# function to solve the SP model
def solve_SP_model(model: gp.Model,
                   chosenNodesVisitedByTruck: dict,
                   chosenArcsVisitedByTruck: dict,
                   arcsVisitedByDrone: dict,
                   truckArrivingTime: dict,
                   droneArrivingTime: dict,
                   efficientSolvingGurobi: bool,
                   graph: nx.DiGraph,
                   droneSpeed: int,
                   t1: time.time):

    # traveling time of truck and drone
    # get the traveling times (dict) --> (arc) : weight
    travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
    travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}

    # deactivate any automatic cuts, heuristics and presolve Gurobi techniques
    if efficientSolvingGurobi == False:
        model.setParam('Cuts', 0)
        model.setParam('ImpliedCuts', 0)
        model.setParam('CliqueCuts', 0)
        model.setParam('Heuristics', 0)
        model.setParam('Presolve', 0)
    # set Time limit
    model.setParam('TimeLimit', 60*60)

    def cb(model, where):
        if where == GRB.Callback.MIP:
            if time.time() - t1 > 3600:
                model.terminate()

    # solve the model
    model.optimize(callback = cb)


    if model.status == GRB.INFEASIBLE:
        return False

    if model.status == GRB.INTERRUPTED:
        result = {}
        return result
    # solution dicts of the model
    chosenArcsVisitedByDrone = {}  # (startNode, endNode, drone, droneTrip, level): 1
    resultTruckArrivingTime = {}  # level: value > 0
    resultDroneArrivingTime = {}  # (endNode, droneTrip, level): value > 0

    # solution graph
    H = nx.DiGraph()
    # define the color of the nodes. red = truck node. green = drone node.
    colors = {}
    # the depot
    H.add_node(0)
    colors[0] = 'red'
    # add the truck nodes to the solution graph
    for (node, level) in chosenNodesVisitedByTruck.keys():
        if chosenNodesVisitedByTruck[(node, level)] > 0.99:
            H.add_node(node)
            colors[node] = 'red'

    # add the drone nodes to the subdigraph
    for node in graph.nodes():
        if node not in H:
            H.add_node(node)
            colors[node] = 'green'

    # add the truck edges to the solution graph
    for (startNode, endNode, level) in chosenArcsVisitedByTruck.keys():
        if chosenArcsVisitedByTruck[(startNode, endNode, level)] > 0.99:
            H.add_edge(startNode, endNode, weight=travelingTimeTruck[(startNode, endNode)])

    # add the drone edges to the solution graph
    for (startNode, endNode, drone, droneTrip, level) in arcsVisitedByDrone.keys():
        if arcsVisitedByDrone[(startNode, endNode, drone, droneTrip, level)].X > 0.99:
            chosenArcsVisitedByDrone[(startNode, endNode, drone, droneTrip, level)] = 1
            H.add_edge(startNode, endNode, weight=travelingTimeDrone[(startNode, endNode)])
            H.add_edge(endNode, startNode, weight=travelingTimeDrone[(startNode, endNode)])
    # get the truck arriving time in each level
    for l in truckArrivingTime.keys():
        if truckArrivingTime[l].X > 0:
            resultTruckArrivingTime[l] = truckArrivingTime[l].X
    # get the drone arriving time in each level in each droneTrip
    for (endNode, droneTrip, level) in droneArrivingTime.keys():
        if droneArrivingTime[(endNode, droneTrip, level)].X > 0:
            resultDroneArrivingTime[(endNode, droneTrip, level)] = droneArrivingTime[(endNode, droneTrip, level)].X

    result = {}
    result['nodesColors'] = colors #red = Truck, green = Drone
    result['droneArcs'] = chosenArcsVisitedByDrone
    result['truckArrivingTimes'] = resultTruckArrivingTime
    result['droneArrivingTimes'] = resultDroneArrivingTime
    result['objectiveValue' ] = model.objVal

    return result


# The LBBD algorithm to solve the MTRPD
def solve_model_LBBD(instancePath: str,
                     droneSpeed: int,
                     numberOfDrones: int,
                     numberOfTrips: int,
                     largeNumber: int,
                     efficientSolvingGurobi: bool,
                     relax: bool,
                     optGapStop: float):
    instanceLocations, instanceArcs, graph = read_instance_file(instancePath)
    travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
    travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}
    flightEndurance = np.ceil(2 * max(travelingTimeDrone.values()))

    t1 = time.time()
    t3 = time.time()

    status = '' # GRB.INFEASIBLE : 3, GRB.OPTIMAL : 2, GRB.TIME_LIMIT : 9
    cpuMP = {}
    cpuSP = {}
    resultNodesTruck = {}
    resultArcsTruck = {}
    resultArcsDroneMP = {}
    resultArcsDroneSP = {}
    resultArrivingTimeTruckMP = {}
    resultArrivingTimeDroneMP = {}
    resultArrivingTimeTruckSP = {}
    resultArrivingTimeDroneSP = {}
    bounds = {}
    optGap = 100
    optCuts = 0

    # build the MP model
    modelMP, constraintsMP, objectiveValueMP, nodesVisitedByTruckMP, arcsVisitedByTruckMP, arcsVisitedByDroneMP, truckArrivingTimeMP, droneArrivingTimeMP = \
        create_MTRPD_MP(graph, numberOfDrones, numberOfTrips, flightEndurance, droneSpeed, relax)

    # initialization
    numberOfLevels = - 1
    U = GRB.INFINITY
    it = 1
    flag = True

    while flag:
        if round(U, 5)*optGapStop <= round(numberOfLevels, 5):
            result = {}

            result['status'] = status
            result['cpuMP'] = cpuMP
            result['cpuSP'] = cpuSP
            result['cpu-totalTime'] = time.time() - t1
            result['truckNodes'] = resultNodesTruck
            result['truckArcs'] = resultArcsTruck
            result['droneArcsMP'] = resultArcsDroneMP
            result['droneArcsSP'] = resultArcsDroneSP
            result['truckArrivingTimesMP'] = resultArrivingTimeTruckMP
            result['droneArrivingTimesMP'] = resultArrivingTimeDroneMP
            result['truckArrivingTimesSP'] = resultArrivingTimeTruckSP
            result['droneArrivingTimesSP'] = resultArrivingTimeDroneSP
            result['bounds'] = bounds
            result['optGap'] = round((U - numberOfLevels)/U * 100, 4)
            result['optCuts'] = optCuts

            return result

        solutionMP = solve_MP_model(modelMP,
                              nodesVisitedByTruckMP,
                              arcsVisitedByTruckMP,
                              arcsVisitedByDroneMP,
                              truckArrivingTimeMP,
                              droneArrivingTimeMP,
                              efficientSolvingGurobi,
                              graph,
                              droneSpeed,
                              t1)
        # STEP 1 solve MP
        if solutionMP == False:
            # the MP is infeasible
            status = GRB.INFEASIBLE
            result = {}
            result['status'] = status
            result['cpuMP'] = cpuMP
            result['cpuSP'] = cpuSP
            result['cpu-totalTime'] = time.time() - t1
            result['truckNodes'] = resultNodesTruck
            result['truckArcs'] = resultArcsTruck
            result['droneArcsMP'] = resultArcsDroneMP
            result['droneArcsSP'] = resultArcsDroneSP
            result['truckArrivingTimesMP'] = resultArrivingTimeTruckMP
            result['droneArrivingTimesMP'] = resultArrivingTimeDroneMP
            result['truckArrivingTimesSP'] = resultArrivingTimeTruckSP
            result['droneArrivingTimesSP'] = resultArrivingTimeDroneSP
            result['bounds'] = bounds
            result['optGap'] = optGap
            result['optCuts'] = optCuts
            return result

        else:
            # Time needed to solve the Master Problem:
            if it == 1:
                t2 = time.time()
                cpuMP[it] = int(t2-t1)
            else:
                t2 = time.time()
                cpuMP[it] = int(t2-t3)

            if modelMP.status == GRB.TIME_LIMIT  or modelMP.status == GRB.INTERRUPTED:
                # time limit reached
                status = GRB.TIME_LIMIT
                result = {}

                bounds[it] = {}
                try:
                    bounds[it]['MPObjectiveValue'] = solutionMP['objectiveValue']
                    bounds[it]['optGap'] = round(((U - solutionMP['objectiveValue']) / U) * 100, 4)
                    result['optGap'] = round(((U - solutionMP['objectiveValue']) / U) * 100, 4)
                except:
                    bounds[it]['MPObjectiveValue'] = GRB.INFINITY
                    result['optGap'] = round(((U - numberOfLevels) / U) * 100, 4)

                bounds[it]['SPObjectiveValue'] = GRB.INFINITY
                bounds[it]['globalUpperBound'] = U
                result['status'] = status
                result['cpuMP'] = cpuMP
                result['cpuSP'] = cpuSP
                result['cpu-totalTime'] = time.time() - t1
                result['truckNodes'] = resultNodesTruck
                result['truckArcs'] = resultArcsTruck
                result['droneArcsMP'] = resultArcsDroneMP
                result['droneArcsSP'] = resultArcsDroneSP
                result['truckArrivingTimesMP'] = resultArrivingTimeTruckMP
                result['droneArrivingTimesMP'] = resultArrivingTimeDroneMP
                result['truckArrivingTimesSP'] = resultArrivingTimeTruckSP
                result['droneArrivingTimesSP'] = resultArrivingTimeDroneSP
                result['bounds'] = bounds
                result['optCuts'] = optCuts
                return result
            else:
                # MP solved to optimality within the time limit
                status = GRB.OPTIMAL

                resultNodesTruck[it] = dict(sorted({key:value for (key,value) in solutionMP['truckNodes'].items() if value > 0}.items(),
                                                   key=lambda t: t[0][1], reverse=True))
                resultArcsTruck[it] = dict(sorted({key:value for (key,value) in solutionMP['truckArcs'].items() if value > 0}.items(),
                                                   key = lambda t: t[0][2], reverse = True))
                resultArcsDroneMP[it] = dict(sorted({key:value for (key,value) in solutionMP['droneArcs'].items() if value > 0}.items(),
                                                   key = lambda t: t[0][3], reverse = True))
                resultArrivingTimeTruckMP[it] = dict(sorted({key:value for (key,value) in solutionMP['truckArrivingTimes'].items() if value > 0}.items(),
                                                   key = lambda t: t[0], reverse = True))
                resultArrivingTimeDroneMP1 = dict(sorted({key:value for (key,value) in solutionMP['droneArrivingTimes'][0].items() if value > 0}.items(),
                                                   key = lambda t: t[0][1], reverse = True))
                resultArrivingTimeDroneMP2 = dict(sorted({key:value for (key,value) in solutionMP['droneArrivingTimes'][1].items() if value > 0}.items(),
                                                   key = lambda t: t[0][2]))
                resultArrivingTimeDroneMP3 = dict(sorted({key:value for (key,value) in solutionMP['droneArrivingTimes'][2].items() if value > 0}.items(),
                                                   key = lambda t: t[0][1], reverse = True))
                resultArrivingTimeDroneMP[it] = {}
                resultArrivingTimeDroneMP[it]['ArrivingTimeDroneMP1'] = resultArrivingTimeDroneMP1
                resultArrivingTimeDroneMP[it]['ArrivingTimeDroneMP2'] = resultArrivingTimeDroneMP2
                resultArrivingTimeDroneMP[it]['ArrivingTimeDroneMP3'] = resultArrivingTimeDroneMP3
                #plot_graph(solutionGraphMP, instanceLocations, nodeColorsMP)

                #set the lower bound
                numberOfLevels = solutionMP['objectiveValue']
                bounds[it] = {}
                bounds[it]['MPObjectiveValue'] = numberOfLevels
                #STEP 2 create and solve SP
                modelSP, constraintsSP, arcsVisitedByDroneSP, truckArrivingTimeSP, droneArrivingTimeSP = \
                    create_MTRPD_SP(graph,
                                    numberOfDrones,
                                    numberOfTrips,
                                    largeNumber,
                                    flightEndurance,
                                    droneSpeed,
                                    solutionMP['truckNodes'],
                                    solutionMP['truckArcs'],
                                    relax)
                solutionSP = solve_SP_model(modelSP,
                                      solutionMP['truckNodes'],
                                      solutionMP['truckArcs'],
                                      arcsVisitedByDroneSP,
                                      truckArrivingTimeSP,
                                      droneArrivingTimeSP,
                                      efficientSolvingGurobi,
                                      graph,
                                      droneSpeed,
                                      t1)
                if solutionSP == False:
                    #add a feasibility cut to the MP
                    constraintsMP['(feasibilityCut)_{}'.format(it)] = modelMP.addConstr(
                        sum(
                            (1 - nodesVisitedByTruckMP[(endNode, level)])
                            for (endNode, level) in solutionMP['truckNodes'].keys() if solutionMP['truckNodes'][(endNode, level)] > 0.99
                        )
                        +
                        sum(
                            nodesVisitedByTruckMP[(endNode, level)]
                            for (endNode, level) in solutionMP['truckNodes'].keys() if solutionMP['truckNodes'][(endNode, level)] == 0
                        )
                        >= 1
                        ,
                        name='(feasibilityCut)_{}'.format(it)
                    )
                    modelMP.update()
                    # uncomment the following command to export the model to a text file
                    # modelMP.write(
                    #     "MTRPD_LBBD_MP_[Instance = {0}, Drones = {1}, Allowed Nr. of Drone Trips = {2}, Flight Range = {3}].lp".format(
                    #         graph, numberOfDrones, numberOfTrips, flightEndurance))
                    it += 1
                else:
                    # Time needed to solve the Sub Problem:
                    t3 = time.time()
                    cpuSP[it] = int(t3 - t2)
                    if modelSP.status == GRB.TIME_LIMIT or modelSP.status == GRB.INTERRUPTED:
                        # time limit reached
                        try:
                            lowestUB = solutionSP['objectiveValue' ]
                            U = min(U, lowestUB)
                            bounds[it]['SPObjectiveValue'] = lowestUB
                            bounds[it]['globalUpperBound'] = U
                            bounds[it]['optGap'] = round(((U - numberOfLevels) / U) * 100, 4)
                        except:
                            bounds[it]['SPObjectiveValue'] = GRB.INFINITY
                            bounds[it]['globalUpperBound'] = U
                            bounds[it]['optGap'] = round(((U - numberOfLevels) / U) * 100, 4)

                        status = GRB.TIME_LIMIT
                        result = {}
                        result['status'] = status
                        result['cpuMP'] = cpuMP
                        result['cpuSP'] = cpuSP
                        result['cpu-totalTime'] = time.time() - t1
                        result['truckNodes'] = resultNodesTruck
                        result['truckArcs'] = resultArcsTruck
                        result['droneArcsMP'] = resultArcsDroneMP
                        result['droneArcsSP'] = resultArcsDroneSP
                        result['truckArrivingTimesMP'] = resultArrivingTimeTruckMP
                        result['droneArrivingTimesMP'] = resultArrivingTimeDroneMP
                        result['truckArrivingTimesSP'] = resultArrivingTimeTruckSP
                        result['droneArrivingTimesSP'] = resultArrivingTimeDroneSP
                        result['bounds'] = bounds
                        result['optGap'] = (U - numberOfLevels) / U * 100
                        result['optCuts'] = optCuts
                        return result
                    else:
                        #SP solved to optimality within the time limit
                        status = GRB.OPTIMAL

                        resultArcsDroneSP[it] = dict(sorted({key:value for (key,value) in solutionSP['droneArcs'].items() if value > 0}.items(),
                                                          key=lambda t: t[0][4], reverse=True))
                        resultArrivingTimeTruckSP[it] = dict(sorted({key:value for (key,value) in solutionSP['truckArrivingTimes'].items() if value > 0}.items(),
                                                                  key = lambda t:t[0], reverse= True))
                        resultArrivingTimeDroneSP[it] =  dict(sorted({key:value for (key,value) in solutionSP['droneArrivingTimes'].items() if value > 0}.items(),
                                                                   key=lambda t: t[0][2], reverse=True))
                        #set the plot
                        lowestUB = solutionSP['objectiveValue' ]
                        U = min(U, lowestUB)
                        bounds[it]['SPObjectiveValue'] = lowestUB
                        bounds[it]['globalUpperBound'] = U
                        bounds[it]['optGap'] = round(((U - numberOfLevels) / U) * 100, 4)

                        #add the optimality Cut
                        constraintsMP['(optimalityCut)_{}'.format(it)] = modelMP.addConstr(
                            objectiveValueMP['eta']
                            >=
                            lowestUB
                            *(
                            1
                            -
                            sum(
                                (1 - nodesVisitedByTruckMP[(endNode, level)])
                                for (endNode, level) in solutionMP['truckNodes'].keys() if solutionMP['truckNodes'][(endNode, level)] > 0.99
                            )
                            -
                            sum(
                                nodesVisitedByTruckMP[(endNode, level)]
                                for (endNode, level) in solutionMP['truckNodes'].keys() if solutionMP['truckNodes'][(endNode, level)] < 0.01
                            )
                            )
                            ,
                            name='(optimalityCut)_{}'.format(it)
                        )
                        optCuts = optCuts + 1
                        modelMP.update()
                        # uncomment the following command to export the model to a text file
                        # modelMP.write(
                        #     "MTRPD_LBBD_MP_[Instance = {0}, Drones = {1}, Allowed Nr. of Drone Trips = {2}, Flight Range = {3}].lp".format(
                        #         graph, numberOfDrones, numberOfTrips, flightEndurance))
                        t3 = time.time()
                        if  t3 - t1 > 3600:
                            # general time limit (3600) reached
                            status = GRB.TIME_LIMIT
                            result = {}
                            result['status'] = status
                            result['cpuMP'] = cpuMP
                            result['cpuSP'] = cpuSP
                            result['cpu-totalTime'] = time.time() - t1
                            result['truckNodes'] = resultNodesTruck
                            result['truckArcs'] = resultArcsTruck
                            result['droneArcsMP'] = resultArcsDroneMP
                            result['droneArcsSP'] = resultArcsDroneSP
                            result['truckArrivingTimesMP'] = resultArrivingTimeTruckMP
                            result['droneArrivingTimesMP'] = resultArrivingTimeDroneMP
                            result['truckArrivingTimesSP'] = resultArrivingTimeTruckSP
                            result['droneArrivingTimesSP'] = resultArrivingTimeDroneSP
                            result['bounds'] = bounds
                            result['optGap'] = (U - numberOfLevels) / U * 100
                            result['optCuts'] = optCuts
                            return result
                        it += 1


def main():

    # general parameters
    # droneSpeed := the drone to truck speed ratio
    droneSpeed = 2
    numberOfDrones = 1  # {1, 2, 3}
    numberOfTrips = 1 # {1, 2, 3}
    optGapStop = 1 # optGapStop must be out of [0, 1]
    largeNumber = 1000000
    efficientSolvingGurobi = False
    relax = False


    # solve the instances, depending on the type
    results = {}

    # uniform centroid - 11 nodes
    for i in range(10):
        instancePath = r"Instances\uniform-centroid-1" + str(i) + "-n11.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        results[instance] = solve_model_LBBD(instancePath,
                                             droneSpeed,
                                             numberOfDrones,
                                             numberOfTrips,
                                             largeNumber,
                                             efficientSolvingGurobi,
                                             relax,
                                             optGapStop)

    # uniform origin - 11 nodes
    for i in range(10):
        instancePath = r"Instances\uniform-origin-11" + str(i) + "-n11.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        results[instance] = solve_model_LBBD(instancePath,
                                             droneSpeed,
                                             numberOfDrones,
                                             numberOfTrips,
                                             largeNumber,
                                             efficientSolvingGurobi,
                                             relax,
                                             optGapStop)
    # clustered centroid - 11 nodes
    for i in range(10):
        instancePath = r"Instances\clustered-centroid-21" + str(i) + "-n11.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        results[instance] = solve_model_LBBD(instancePath,
                                             droneSpeed,
                                             numberOfDrones,
                                             numberOfTrips,
                                             largeNumber,
                                             efficientSolvingGurobi,
                                             relax,
                                             optGapStop)

    #clustered origin - 11 nodes
    for i in range(10):
        instancePath = r"Instances\clustered-origin-31" + str(i) + "-n11.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        results[instance] = solve_model_LBBD(instancePath,
                                             droneSpeed,
                                             numberOfDrones,
                                             numberOfTrips,
                                             largeNumber,
                                             efficientSolvingGurobi,
                                             relax,
                                             optGapStop)


    print(results)



if __name__ == '__main__':
    main()