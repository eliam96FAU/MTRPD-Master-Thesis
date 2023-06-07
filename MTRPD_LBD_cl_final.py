# gurobi version 10.0.0
import csv
import json

import gurobipy as gp
import math
from gurobipy import GRB
import networkx as nx
import numpy as np
from matplotlib import pyplot as plt
import collections
import pandas as pd
import copy as copy
import time
import export_import_Results
from scipy.stats import binom



# function to create the graph
def create_graph(nodeList, arcList):
    # Create graph
    graph = nx.DiGraph()
    # add nodes
    graph.add_nodes_from(nodeList)
    # add arcs
    graph.add_weighted_edges_from(arcList)
    return graph


# G -> Graph;
# K -> Number of available drones;
# Q -> Number of possible trips;
# M -> large number;
# E -> flight endurance of the drones;
# S -> Drone Speed
def create_MTRPD_MP(graph, L, K, Q, E, S, relax):
    # Initialize the model
    model = gp.Model()
    # in case we don't need output
    # model.setParam('OutputFlag', False)

    # number of Levels / customers to be visited in the backward order
    # L = graph.number_of_nodes() - 1

    # get the traveling times (dict) --> (arc) : weight
    travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
    travelingTimeDrone = {arc: int(time / S) for (arc, time) in travelingTimeTruck.items()}

    # nodes - L dict -> (node, level): Gurobi variables
    nodesVisitedByTruck = {}

    # arcs dict -> (arc, level) : Gurobi varialbes
    arcsVisitedByTruck = {}
    # arcs dict -> (arc, drone, level) : Gurobi varialbes
    arcsVisitedByDrone = {}

    # Arriving time of Truck in the level l
    # L dict -> level : Gurobi variable
    truckArrivingTime = {}

    # Arriving time of a drone at a customer j in the level l in its qth trip
    # L-customers'-trip dict -> L-customers'-trip : Gurobi variable
    droneArrivingTime1 = {}
    droneArrivingTime2 = {}
    droneArrivingTime3 = {}
    lambda1Vars = {}
    lambda2Vars = {}
    lambda3Vars = {}
    lambda4Vars = {}

    objectiveValue = {'eta': model.addVar(vtype=GRB.CONTINUOUS, lb=0, name='eta')}
    # Gurobi Variables:
    # binary node variable to assign a customer to the truck at level l.
    # x_i^l == 1 if node i outOf T is visited in the backward position l
    for node in graph.nodes():
        if node == 0:
            for level in range(L + 1, 0, -1):
                nodesVisitedByTruck[(node, level)] = \
                    model.addVar(
                        vtype=GRB.BINARY,
                        lb=0,
                        ub=1,
                        name="x_{0}^{1}".format(node, level)
                    )
        else:
            for level in range(L, 0, -1):
                nodesVisitedByTruck[(node, level)] = \
                    model.addVar(
                        vtype=GRB.BINARY,
                        lb=0,
                        ub=1,
                        name="x_{0}^{1}".format(node, level)
                    )

    for arc in graph.edges():
        # In the first level (L) we have only the arcs from the depot to all other customers
        if arc[0] == 0:
            for level in range(L, 0, -1):
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
            for level in range(L - 1, 0, -1):
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
        if arc[0] == 0:
            if arc[1] != 0:
                for level in range(L + 1, 0, -1):
                    for drone in range(1, K + 1, 1):
                        arcsVisitedByDrone[(arc[0], arc[1], drone, level)] = \
                            model.addVar(
                                vtype=GRB.BINARY,
                                lb=0,
                                ub=1,
                                name="z_{0}_{1}_{2}^{3}".format(arc[0], arc[1], drone, level)
                            )
        else:
            if arc[1] != 0:
                for level in range(L, 0, -1):
                    for drone in range(1, K + 1, 1):
                        arcsVisitedByDrone[(arc[0], arc[1], drone, level)] = \
                            model.addVar(
                                vtype=GRB.BINARY,
                                lb=0,
                                ub=1,
                                name="z_{0}_{1}_{2}^{3}".format(arc[0], arc[1], drone, level)
                            )
    for level in range(L, 0, -1):
        # non-negative continuous variable representing the arrivel time at the l-th last customer served by the truck
        # t^l
        truckArrivingTime[level] = model.addVar(
            vtype=GRB.CONTINUOUS,
            lb=0,
            name="t^{0}".format(level)
        )
        for endNode in graph.nodes():
            if endNode != 0:
                droneArrivingTime1[(endNode, level)] = model.addVar(
                    vtype=GRB.CONTINUOUS,
                    lb=0,
                    name="pi1_{0}^{1}".format(endNode, level)
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

    for level in range(L + 1, 0, -1):
        # non-negative continuous variable representing the arrivel time at node j outOf D served by a drone in its t-th trip
        # while truck waiting at the lth last customer
        for endNode in graph.nodes():
            if endNode != 0:
                droneArrivingTime3[(endNode, level)] = model.addVar(
                    vtype=GRB.CONTINUOUS,
                    lb=0,
                    name="pi3_{0}^{1}".format(endNode, level)
                )

    for launchNode in graph.nodes():
        # non-negative continuous variable representing the previous trips performed by drone k at the launch location
        for drone in range(1, K + 1, 1):
            for droneTrip in range(1, Q + 1, 1):
                droneArrivingTime2[(launchNode, drone, droneTrip)] = model.addVar(
                    vtype=GRB.CONTINUOUS,
                    lb=0,
                    name="pi2_{0}_{1}_{2}".format(launchNode, drone, droneTrip)
                )

    # truck constraints:
    # todo: add a dict for the constraints to track those better
    constraints = {i: [] for i in range(1, 14)}
    # (1) the objective
    constraints[1].append(model.addConstr(
        objectiveValue['eta']
        >=
        sum(
            truckArrivingTime[level]
            for level in range(L, 0, -1)
        )
        +
        # drones
        sum(
            droneArrivingTime1[(endNode, level)]
            for endNode in graph.nodes() if endNode != 0
            for level in range(L, 0, -1)
        )
        +
        sum(
            droneArrivingTime2[(launchNode, drone, droneTrip)]
            for launchNode in graph.nodes()
            for drone in range(1, K + 1, 1)
            for droneTrip in range(1, Q + 1, 1)
        )
        +
        sum(
            droneArrivingTime3[(endNode, level)]
            for endNode in graph.nodes() if endNode != 0
            for level in range(L + 1, 0, -1)
        ),
        name="(1)"
    ))
    # (2a) the truck ends the route at a customer in the level 1 (the last level)
    # Definition: startNode is the starting node of an arc
    constraints[2].append(model.addConstr(
        sum(
            nodesVisitedByTruck[(startNode, 1)]
            for startNode in range(1, len(graph.nodes))
        )
        == 1,
        name='(2a)'
    ))
    # (2b) the depot node shouldn't be visited by the truck at level 1
    model.addConstr(
        nodesVisitedByTruck[(0, 1)] == 0,
        name='(2b)'
    )
    # (3) The truck leaves the depot to a node j in one of the levels l
    # Definition: endNode is the ending node of an arc
    constraints[3].append(model.addConstr(
        sum(
            arcsVisitedByTruck[(0, endNode, level)]
            for endNode in graph.nodes() if endNode != 0
            for level in range(L, 0, -1)
        )
        == 1,
        name='(3)'
    ))
    # (4) The connection between the nodes and arcs visited by the truck. - outbound
    # i.e. if a node i is visited by the truck in the level l+1 , then the truck needs to use one of the arcs in the level l to leave this node i
    # (4a) customers
    for startNode in graph.nodes():
        for level in range(L - 1, 0, -1):
            if startNode != 0:
                constraints[4].append(model.addConstr(
                    sum(
                        arcsVisitedByTruck[(startNode, endNode, level)]
                        for endNode in graph.nodes() if endNode != 0 and endNode != startNode
                    )
                    == nodesVisitedByTruck[(startNode, level + 1)],
                    name='(4a_[i = {0}, l = {1}])'.format(startNode, level)
                ))
    # (4b) depot
    for level in range(L, 0, -1):
        constraints[4].append(model.addConstr(
            sum(
                arcsVisitedByTruck[(0, endNode, level)]
                for endNode in graph.nodes() if endNode != 0
            )
            == nodesVisitedByTruck[(0, level + 1)],
            name='(4b_[i = {0}, l = {1}])'.format(0, level)
        ))
    # (5) The connection between the nodes and arcs visited by the truck. - inbound
    for endNode in graph.nodes():
        if endNode != 0:
            for level in range(L - 1, 0, -1):
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
                arcsVisitedByTruck[(0, endNode, L)] == nodesVisitedByTruck[(endNode, L)],
                name='(6_[j = {0}])'.format(endNode)
            ))
    # (7a) all nodes/customers are visited either by the truck or the drone
    for endNode in graph.nodes():
        if endNode != 0:
            constraints[7].append(model.addConstr(
                # customers being serviced by the truck
                sum(
                    nodesVisitedByTruck[(endNode, level)]
                    for level in range(L, 0, -1)
                )
                +
                # customers being serviced a drone launched from a customer node
                sum(
                    arcsVisitedByDrone[(startNode, endNode, drone, level)]
                    for startNode in graph.nodes() if startNode != endNode and startNode != 0
                    for drone in range(1, K + 1, 1)
                    for level in range(L, 0, -1)
                )
                +
                # customers being serviced a drone launched from the depot
                sum(
                    arcsVisitedByDrone[(0, endNode, drone, level)]
                    for drone in range(1, K + 1, 1)
                    for level in range(L + 1, 0, -1)
                )
                == 1,
                name='(7a_[i = {0}])'.format(endNode)
            ))
    # (7b) the number of customers visited by the truck is = the observed number of Levels
    constraints[7].append(model.addConstr(
        # customers being serviced by the truck
        sum(
            nodesVisitedByTruck[(launchNode, level)]
            for level in range(L, 0, -1)
            for launchNode in graph.nodes() if launchNode != 0
        )
        == L,
        name='(7b)'
    ))
    # (8a) drone trips constraints when launch node is a customer node
    for level in range(L, 0, -1):
        for startNode in graph.nodes():
            if startNode != 0:
                for drone in range(1, K + 1, 1):
                    constraints[8].append(model.addConstr(
                        sum(
                            arcsVisitedByDrone[(startNode, endNode, drone, level)]
                            for endNode in graph.nodes if endNode != startNode and endNode != 0
                        )
                        <= Q * nodesVisitedByTruck[(startNode, level)],
                        name='(8a_[i = {0}, k = {1}, l = {2}])'.format(startNode, drone, level)
                    ))
    # (8b) drone trips constraints when launch node is the depot
    for level in range(L + 1, 0, -1):
        for drone in range(1, K + 1, 1):
            constraints[8].append(model.addConstr(
                sum(
                    arcsVisitedByDrone[(0, endNode, drone, level)]
                    for endNode in graph.nodes if endNode != 0
                )
                <= Q * nodesVisitedByTruck[(0, level)],
                name='(8b_[i = {0}, k = {1}, l = {2}])'.format(0, drone, level)
            ))
    # (9) Arriving time of the truck in the level l
    for level in range(L, 0, -1):
        for drone in range(1, K + 1, 1):
            # first step of the truck
            if level == L:
                constraints[9].append(model.addConstr(
                    # t^l
                    truckArrivingTime[level]
                    >=
                    # the traveling time of the truck from the depot to a customer
                    # + the waiting time of the  truck until the last drone comes back from its last trip at level L + 1
                    sum(
                        travelingTimeTruck[(0, endNode)] * arcsVisitedByTruck[(0, endNode, level)]
                        +
                        (travelingTimeDrone[(0, endNode)] + travelingTimeDrone[(endNode, 0)]) *
                        arcsVisitedByDrone[(0, endNode, drone, level + 1)]
                        for endNode in graph.nodes() if endNode != 0
                    )
                    ,
                    name='(9_[l = {0}])'.format(level)
                ))
            # starting from the second last step of the truck
            else:
                constraints[9].append(model.addConstr(
                    # t^l
                    truckArrivingTime[level]
                    >=
                    # previous traveling (and waiting time)
                    # we separate the following two sums because in level L there are only arcs out of the depot
                    # truck arcs out of the depot 0
                    sum(
                        travelingTimeTruck[(0, endNode)] * arcsVisitedByTruck[(0, endNode, previousLevel)]
                        for endNode in graph.nodes() if endNode != 0
                        for previousLevel in range(L, level, -1)
                    )
                    +
                    # duration of previous levels of the truck
                    # arcs out of nodes other than the depot 0
                    sum(
                        travelingTimeTruck[(startNode, endNode)] * arcsVisitedByTruck[
                            (startNode, endNode, previousLevel)]
                        for startNode in graph.nodes() if startNode != 0
                        for endNode in graph.nodes() if endNode != startNode and endNode != 0
                        for previousLevel in range(L - 1, level, -1)
                    )
                    +
                    # duration of drone trips at level L + 1 from the depot
                    sum(
                        (travelingTimeDrone[(0, endNode)] + travelingTimeDrone[(endNode, 0)]) *
                        arcsVisitedByDrone[(0, endNode, drone, L + 1)]
                        for endNode in graph.nodes() if endNode != 0
                    )
                    +
                    # duration of drone trips in previous levels but level L + 1
                    sum(
                        (travelingTimeDrone[(startNode, endNode)] + travelingTimeDrone[(endNode, startNode)]) *
                        arcsVisitedByDrone[(startNode, endNode, drone, previousLevel)]
                        for startNode in graph.nodes()  # if startNode != 0
                        for endNode in graph.nodes() if endNode != startNode and endNode != 0
                        for previousLevel in range(L, level, -1)
                    )
                    +
                    # current traveling time of the truck
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
            for level in range(L, 0, -1):
                worstCaseTruckArrivingTime = (len(graph.nodes()) - 1) * max(travelingTimeTruck.values())
                constraints[10].append(model.addConstr(
                    droneArrivingTime1[(endNode, level)]
                    ==
                    lambda4Vars[(endNode, level)] * worstCaseTruckArrivingTime,
                    name='(10a_[j = {0}, l = {1}])'.format(endNode, level)
                ))
                constraints[10].append(model.addConstr(
                    truckArrivingTime[level]
                    ==
                    (lambda3Vars[(endNode, level)] + lambda4Vars[(endNode, level)]) * worstCaseTruckArrivingTime,
                    name='(10b_[j = {0}, l = {1}])'.format(endNode, level)
                ))
                constraints[10].append(model.addConstr(
                    sum(arcsVisitedByDrone[(launchNode, endNode, drone, level)]
                        for launchNode in graph.nodes() if launchNode != endNode
                        for drone in range(1, K + 1, 1)
                        )
                    ==
                    (lambda2Vars[(endNode, level)] + lambda4Vars[(endNode, level)]),
                    name='(10c_[j = {0}, l = {1}])'.format(endNode, level)
                ))
                constraints[10].append(model.addConstr(
                    (lambda1Vars[(endNode, level)] + lambda2Vars[(endNode, level)] + lambda3Vars[(endNode, level)] +
                     lambda4Vars[(endNode, level)])
                    ==
                    1,
                    name='(10d_[j = {0}, l = {1}])'.format(endNode, level)
                ))
    # (11a) Arriving time of a drone in its qth trip at customer j in the level l from a customer
    for launchNode in graph.nodes():
        if launchNode != 0:
            for drone in range(1, K + 1, 1):
                for droneTrip in range(1, Q + 1, 1):
                    tdMin = min([2 * travelingTimeDrone[(launchNode, endNode)] for endNode in graph.nodes() if
                                 endNode != 0 and endNode != launchNode])
                    constraints[11].append(model.addConstr(
                        droneArrivingTime2[(launchNode, drone, droneTrip)]
                        >=
                        (sum(arcsVisitedByDrone[(launchNode, endNode, drone, level)]
                             for endNode in graph.nodes() if endNode != 0 and endNode != launchNode
                             for level in range(L, 0, -1)) - droneTrip) * tdMin,
                        name='(11a_[i = {0}, k = {1}, q = {2}])'.format(launchNode, drone, droneTrip)
                    ))
    # (11b) Arriving time of a drone in its qth trip at customer j in the level l from the depot
    for drone in range(1, K + 1, 1):
        for droneTrip in range(1, Q + 1, 1):
            tdMin = min([2 * travelingTimeDrone[(0, endNode)] for endNode in graph.nodes() if endNode != 0])
            constraints[11].append(model.addConstr(
                droneArrivingTime2[(0, drone, droneTrip)]
                >=
                (sum(arcsVisitedByDrone[(0, endNode, drone, level)]
                     for endNode in graph.nodes() if endNode != 0
                     for level in range(L + 1, 0, -1)) - droneTrip) * tdMin,
                name='(11b_[i = {0}, k = {1}, q = {2}])'.format(0, drone, droneTrip)
            ))
    # (12a) Arriving time of a drone in its qth trip at customer j in the level l out of {1, ..., L}
    for endNode in graph.nodes():
        if endNode != 0:
            for level in range(L, 0, -1):
                constraints[12].append(model.addConstr(
                    droneArrivingTime3[(endNode, level)]
                    ==
                    sum(travelingTimeDrone[(launchNode, endNode)] * arcsVisitedByDrone[
                        (launchNode, endNode, drone, level)]
                        for launchNode in graph.nodes() if launchNode != endNode
                        for drone in range(1, K + 1, 1)),
                    name='(12a_[j = {0}, l = {1}])'.format(endNode, level)
                ))
    # (12b) Arriving time of a drone in its qth trip at customer j in the level L + 1
    for endNode in graph.nodes():
        if endNode != 0:
            constraints[12].append(model.addConstr(
                droneArrivingTime3[(endNode, L + 1)]
                ==
                sum(travelingTimeDrone[(0, endNode)] * arcsVisitedByDrone[(0, endNode, drone, L + 1)]
                    for drone in range(1, K + 1, 1)),
                name='(12b_[j = {0}, l = {1}])'.format(endNode, L + 1)
            ))
    # (13a) the flight endurance constraint of each drone
    for launchNode in graph.nodes():
        if launchNode != 0:
            for endNode in graph.nodes():
                if endNode != launchNode and endNode != 0:
                    for drone in range(1, K + 1, 1):
                        for droneTrip in range(1, Q + 1, 1):
                            for level in range(L, 0,
                                               -1):  # level L is exculed since it's not considered. It's not possible in this model to start a drone in Level L, since if we send one drone, we would be in level L-1
                                constraints[13].append(model.addConstr(
                                    (travelingTimeDrone[(launchNode, endNode)] + travelingTimeDrone[
                                        (endNode, launchNode)])
                                    * arcsVisitedByDrone[(launchNode, endNode, drone, level)]
                                    <=
                                    E,
                                    name='(13a_[i = {0}, j = {1}, k = {2}, l = {3}])_d'.format(launchNode, endNode,
                                                                                               drone,
                                                                                               level)
                                ))
    # (13b) the flight endurance constraint of each drone
    for endNode in graph.nodes():
        if endNode != 0:
            for drone in range(1, K + 1, 1):
                for droneTrip in range(1, Q + 1, 1):
                    for level in range(L + 1, 0,
                                       -1):  # level L is exculed since it's not considered. It's not possible in this model to start a drone in Level L, since if we send one drone, we would be in level L-1
                        constraints[13].append(model.addConstr(
                            (travelingTimeDrone[(0, endNode)] + travelingTimeDrone[(endNode, 0)])
                            * arcsVisitedByDrone[(0, endNode, drone, level)]
                            <=
                            E,
                            name='(13b_[i = {0}, j = {1}, k = {2}, l = {3}])_d'.format(0, endNode,
                                                                                      drone,
                                                                                      level)
                        ))
    # (0) set the objective: Sum of all traveling times of the truck and the drones
    model.setObjective(
        objectiveValue['eta']
        , GRB.MINIMIZE,
    )

    droneArrivingTime = [droneArrivingTime1, droneArrivingTime2, droneArrivingTime3]

    if relax == True:
        # return ralaxed model
        for key in nodesVisitedByTruck.keys():
            nodesVisitedByTruck[key].vtype = GRB.CONTINUOUS
        for key in arcsVisitedByTruck.keys():
            arcsVisitedByTruck[key].vtype = GRB.CONTINUOUS
        for key in arcsVisitedByDrone.keys():
            arcsVisitedByDrone[key].vtype = GRB.CONTINUOUS
        model.write(
            'models\\' + "relaxed_MTRPD_LBD_cL_MP_[Instance = {0}, Drones = {1}, Allowed Nr. of Drone Trips = {2}, Flight Range = {3}].lp".format(
                graph, K, Q, E))
        return model, constraints, objectiveValue, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime
    else:
        model.write(
            'models\\' + "MTRPD_LBD_cL_MP[Instance = {0}, Drones = {1}, Allowed Nr. of Drone Trips = {2}, Flight Range = {3}].lp".format(
                graph, K, Q, E))
        return model, constraints, objectiveValue, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime


def create_MTRPD_SP(graph, L, K, Q, M, E, S, chosenNodesTruck, chosenArcsTruck, relax):
    # Initialize the model
    model = gp.Model()

    # model.setParam('OutputFlag', False)

    # number of Levels / customers to be visited in the backward order
    # L = graph.number_of_nodes() - 1

    # get the traveling times (dict) --> (arc) : weight
    travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
    travelingTimeDrone = {arc: int(time / S) for (arc, time) in travelingTimeTruck.items()}

    # arcs dict -> arc - level - drone - trip : Gurobi varialbes
    arcsVisitedByDrone = {}

    # Arriving time of Truck in the level l
    # L dict -> level : Gurobi variable
    truckArrivingTime = {}

    # Arriving time of a drone at a customer j in the level l in its qth trip
    # L-customers'-trip dict -> L-customers'-trip : Gurobi variable
    droneArrivingTime = {}

    # Gurobi Variables:
    # binary arc variable on the drone delivery route.
    # f_ijkt^l == 1 if the truck launches drone k to serve node j outOf D in its tth trip, while waiting at node i in level l
    for arc in graph.edges():  # arc[0] != arc[1]
        if arc[0] == 0:
            if arc[1] != 0:
                for level in range(L + 1, 0, -1):
                    for drone in range(1, K + 1, 1):
                        for droneTrip in range(1, Q + 1, 1):
                            arcsVisitedByDrone[(arc[0], arc[1], drone, droneTrip, level)] = \
                                model.addVar(
                                    vtype=GRB.BINARY,
                                    lb=0,
                                    ub=1,
                                    name="f_{0}_{1}_{2}_{3}^{4}".format(arc[0], arc[1], drone, droneTrip, level)
                                )
        else:
            if arc[1] != 0:
                for level in range(L, 0, -1):
                    for drone in range(1, K + 1, 1):
                        for droneTrip in range(1, Q + 1, 1):
                            arcsVisitedByDrone[(arc[0], arc[1], drone, droneTrip, level)] = \
                                model.addVar(
                                    vtype=GRB.BINARY,
                                    lb=0,
                                    ub=1,
                                    name="f_{0}_{1}_{2}_{3}^{4}".format(arc[0], arc[1], drone, droneTrip, level)
                                )
    for level in range(L, 0, -1):
        # non-negative continuous variable representing the arrivel time at the lth last customer served by the truck
        # t^l
        truckArrivingTime[level] = model.addVar(
            vtype=GRB.CONTINUOUS,
            lb=0,
            name="t^{0}".format(level)
        )
    for level in range(L + 1, 0, -1):
        # non-negative continuous variable representing the arrivel time at node j outOf D served by a drone in its tth trip
        # while truck waiting at the lth last customer
        for node in graph.nodes():
            if node != 0:
                for droneTrip in range(1, Q + 1, 1):
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
                    for level in range(L, 0, -1)
                )
                +
                # customers being serviced a drone launched from a customer node
                sum(
                    arcsVisitedByDrone[(startNode, endNode, drone, droneTrip, level)]
                    for startNode in graph.nodes() if startNode != endNode and startNode != 0
                    for drone in range(1, K + 1, 1)
                    for droneTrip in range(1, Q + 1, 1)
                    for level in range(L, 0, -1)
                )
                +
                # customers being serviced a drone launched from the depot
                sum(
                    arcsVisitedByDrone[(0, endNode, drone, droneTrip, level)]
                    for drone in range(1, K + 1, 1)
                    for droneTrip in range(1, Q + 1, 1)
                    for level in range(L + 1, 0, -1)
                )
                == 1,
                name='(19_[i = {0}])'.format(endNode)
            ))
    # (20a) drone trips constraints when launch node is a customer node
    for level in range(L, 0, -1):
        for startNode in graph.nodes():
            if startNode != 0:
                for drone in range(1, K + 1, 1):
                    constraints[20].append(model.addConstr(
                        sum(
                            arcsVisitedByDrone[(startNode, endNode, drone, droneTrip, level)]
                            for endNode in graph.nodes if endNode != startNode and endNode != 0
                            for droneTrip in range(1, Q + 1, 1)
                        )
                        <= Q * chosenNodesTruck[(startNode, level)],
                        name='(20a_[i = {0}, k = {1}, l = {2}])'.format(startNode, drone, level)
                    ))
    # (20b) drone trips constraints when launch node is the depot
    for level in range(L + 1, 0, -1):
        for drone in range(1, K + 1, 1):
            constraints[20].append(model.addConstr(
                sum(
                    arcsVisitedByDrone[(0, endNode, drone, droneTrip, level)]
                    for endNode in graph.nodes if endNode != 0
                    for droneTrip in range(1, Q + 1, 1)
                )
                <= Q * chosenNodesTruck[(0, level)],
                name='(20b_[i = {0}, k = {1}, l = {2}])'.format(0, drone, level)
            ))
    # (21) Arriving time of the truck in the level l
    for level in range(L, 0, -1):
        for drone in range(1, K + 1, 1):
            # first step of the truck
            if level == L:
                constraints[21].append(model.addConstr(
                    # t^l
                    truckArrivingTime[level]
                    >=
                    # the traveling time of the truck from the depot to a customer
                    sum(
                        travelingTimeTruck[(0, endNode)] * chosenArcsTruck[(0, endNode, level)]
                        for endNode in graph.nodes() if endNode != 0)
                    +
                    # the waiting time of the  truck until the last drone comes back from its last trip at level L + 1
                    sum(
                        (travelingTimeDrone[(0, endNode)] + travelingTimeDrone[(endNode, 0)]) *
                        arcsVisitedByDrone[(0, endNode, drone, droneTrip, level + 1)]
                        for endNode in graph.nodes() if endNode != 0
                        for droneTrip in range(1, Q + 1, 1)
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
                    # we separate the following two sums because in level L there are only arcs out of the depot
                    # truck arcs out of the depot 0
                    sum(
                        travelingTimeTruck[(0, endNode)] * chosenArcsTruck[(0, endNode, previousLevel)]
                        for endNode in graph.nodes() if endNode != 0
                        for previousLevel in range(L, level, -1)
                    )
                    +
                    # duration of previous levels of the truck
                    # arcs out of nodes other than the depot 0
                    sum(
                        travelingTimeTruck[(startNode, endNode)] * chosenArcsTruck[
                            (startNode, endNode, previousLevel)]
                        for startNode in graph.nodes() if startNode != 0
                        for endNode in graph.nodes() if endNode != startNode and endNode != 0
                        for previousLevel in range(L - 1, level, -1)
                        # since out of level L there are only truck arcs out of the depot
                    )
                    +
                    # duration of drone trips at level L + 1 from the depot
                    sum(
                        (travelingTimeDrone[(0, endNode)] + travelingTimeDrone[(endNode, 0)]) *
                        arcsVisitedByDrone[(0, endNode, drone, droneTrip, L + 1)]
                        for endNode in graph.nodes() if endNode != 0
                        for droneTrip in range(1, Q + 1, 1)
                        # since out of level L there are only arcs out of the depot
                    )
                    +
                    # duration of previous levels but level L + 1, drone
                    sum(
                        (travelingTimeDrone[(startNode, endNode)] + travelingTimeDrone[(endNode, startNode)]) *
                        arcsVisitedByDrone[(startNode, endNode, drone, droneTrip, previousLevel)]
                        for startNode in graph.nodes()
                        for endNode in graph.nodes() if endNode != startNode and endNode != 0
                        for droneTrip in range(1, Q + 1, 1)
                        for previousLevel in range(L, level, -1)
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

    # (22a) Arriving time of a drone in its qth trip at customer j in the level L + 1
    for endNode in graph.nodes():
        if endNode != 0:
            for drone in range(1, K + 1, 1):
                for droneTrip in range(1, Q + 1, 1):
                    if droneTrip == 1:
                        constraints[22].append(model.addConstr(
                            # pi_j_q^l
                            droneArrivingTime[(endNode, droneTrip, L + 1)]
                            >=
                            # current traveling time to a customer j by the drone
                            travelingTimeDrone[(0, endNode)] * arcsVisitedByDrone[
                                (0, endNode, drone, droneTrip, L + 1)]
                            # turning off the constraint if not visited
                            - M * (1 - arcsVisitedByDrone[(0, endNode, drone, droneTrip, L + 1)]),
                            name='(22a_[j = {0}, k = {1}, q = 1, l = {2}])'.format(endNode, drone, L + 1)
                        ))
                    # number of trips > 1
                    else:
                        constraints[22].append(model.addConstr(
                            # pi_j_q^l
                            droneArrivingTime[(endNode, droneTrip, L + 1)]
                            >=
                            # previous trips traveling time for a drone k
                            sum(
                                (travelingTimeDrone[(0, previousEndNode)] + travelingTimeDrone[
                                    (previousEndNode, 0)])
                                * arcsVisitedByDrone[(0, previousEndNode, drone, previousDroneTrip, L + 1)]
                                for previousEndNode in graph.nodes() if
                                previousEndNode != 0 and previousEndNode != endNode
                                for previousDroneTrip in range(1, droneTrip, 1)
                            )
                            +
                            # current traveling time for a drone k to a customer j
                            travelingTimeDrone[(0, endNode)] * arcsVisitedByDrone[
                                (0, endNode, drone, droneTrip, L + 1)]
                            - M * (1 - arcsVisitedByDrone[(0, endNode, drone, droneTrip, L + 1)]),
                            name='(22a_[j = {0}, k = {1}, q = {2}, l = {3}])'.format(endNode, drone, droneTrip,
                                                                                     L + 1)
                        ))
    # (22b) Arriving time of a drone in its qth trip at customer j in the level l (not L + 1)
    for endNode in graph.nodes():
        if endNode != 0:
            for drone in range(1, K + 1, 1):
                for droneTrip in range(1, Q + 1, 1):
                    for level in range(L, 0, -1):
                        if droneTrip == 1:
                            constraints[22].append(model.addConstr(
                                # pi_j_q^l
                                droneArrivingTime[(endNode, droneTrip, level)]
                                >=
                                truckArrivingTime[level]
                                +
                                # current traveling time to a customer j by the drone from a launch node
                                sum(
                                    travelingTimeDrone[(launchNode, endNode)] * arcsVisitedByDrone[
                                        (launchNode, endNode, drone, droneTrip, level)]
                                    for launchNode in graph.nodes() if launchNode != endNode
                                )
                                - M * (1 - sum(
                                    arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                                    for launchNode in graph.nodes() if launchNode != endNode
                                )),
                                name='(22b_[j = {0}, k = {1}, q = 1, l = {2}])'.format(endNode, drone, level)
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
                                    * arcsVisitedByDrone[
                                        (launchNode, previousEndNode, drone, previousDroneTrip, level)]
                                    for launchNode in graph.nodes() if launchNode != endNode
                                    for previousEndNode in graph.nodes() if
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
                                - M * (1 - sum(
                                    arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                                    for launchNode in graph.nodes() if launchNode != endNode
                                )),
                                name='(22b_[j = {0}, k = {1}, q = {2}, l = {3}])'.format(endNode, drone, droneTrip,
                                                                                         level)
                            ))

    # (23a) customers locations: Each drone is visiting only one node per trip before coming back to the truck
    for launchNode in graph.nodes():
        if launchNode != 0:
            for drone in range(1, K + 1, 1):
                for droneTrip in range(1, Q + 1, 1):
                    constraints[23].append(model.addConstr(
                        sum(
                            arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                            for endNode in graph.nodes() if endNode != launchNode and endNode != 0
                            for level in range(L, 0, -1)
                        )
                        <= 1,
                        name='(23a_[i = {0}, k = {1}, q = {2}])'.format(launchNode, drone, droneTrip)
                    ))
    # (23b) Depot: Each drone is visiting only one node per trip before coming back to the truck
    for drone in range(1, K + 1, 1):
        for droneTrip in range(1, Q + 1, 1):
            constraints[23].append(model.addConstr(
                sum(
                    arcsVisitedByDrone[(0, endNode, drone, droneTrip, level)]
                    for endNode in graph.nodes() if endNode != 0
                    for level in range(L + 1, 0, -1)
                )
                <= 1,
                name='(23b_[i = {0}, k = {1}, q = {2}])'.format(0, drone, droneTrip)
            ))

    # (24a) Customers locations: trip order is ensured
    if Q >= 2:
        for launchNode in graph.nodes():
            if launchNode != 0:
                for drone in range(1, K + 1, 1):
                    for droneTrip in range(1, Q, 1):
                        for level in range(L, 0, -1):
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
                                name='(24a_[i = {0}, k = {1}, q = {2}, l = {3}])'.format(launchNode, drone,
                                                                                         droneTrip,
                                                                                         level)
                            ))
    # (24b) Depot: trip order is ensured
    if Q >= 2:
        for drone in range(1, K + 1, 1):
            for droneTrip in range(1, Q, 1):
                for level in range(L + 1, 0, -1):
                    constraints[24].append(model.addConstr(
                        sum(
                            arcsVisitedByDrone[(0, endNode, drone, droneTrip, level)]
                            for endNode in graph.nodes() if endNode != 0
                        )
                        >=
                        sum(
                            arcsVisitedByDrone[(0, endNode, drone, droneTrip + 1, level)]
                            for endNode in graph.nodes() if endNode != 0
                        ),
                        name='(24b_[i = {0}, k = {1}, q = {2}, l = {3}])'.format(0, drone, droneTrip,
                                                                                 level)
                    ))

    # (25a) Customers locations: the flight endurance constraint of each drone
    for launchNode in graph.nodes():
        if launchNode != 0:
            for endNode in graph.nodes():
                if endNode != launchNode and endNode != 0:
                    for drone in range(1, K + 1, 1):
                        for droneTrip in range(1, Q + 1, 1):
                            for level in range(L, 0, -1):
                                constraints[25].append(model.addConstr(
                                    (travelingTimeDrone[(launchNode, endNode)] + travelingTimeDrone[
                                        (endNode, launchNode)])
                                    * arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                                    <=
                                    E,
                                    name='(25a_[i = {0}, j = {1}, k = {2}, q = {3}, l = {4}])'.format(launchNode,
                                                                                                      endNode,
                                                                                                      drone,
                                                                                                      droneTrip,
                                                                                                      level)
                                ))
    # (25b) Depot: the flight endurance constraint of each drone
    for endNode in graph.nodes():
        if endNode != 0:
            for drone in range(1, K + 1, 1):
                for droneTrip in range(1, Q + 1, 1):
                    for level in range(L + 1, 0, -1):
                        constraints[25].append(model.addConstr(
                            (travelingTimeDrone[(0, endNode)] + travelingTimeDrone[(endNode, 0)])
                            * arcsVisitedByDrone[(0, endNode, drone, droneTrip, level)]
                            <=
                            E,
                            name='(25b_[i = {0}, j = {1}, k = {2}, q = {3}, l = {4}])'.format(0, endNode,
                                                                                              drone, droneTrip,
                                                                                              level)
                        ))

    # (0) set the objective: Sum of all traveling times of the truck and the drones
    model.setObjective(
        # truck
        sum(
            truckArrivingTime[level]
            for level in range(L, 0, -1)
        )
        +
        # drones
        sum(
            droneArrivingTime[(endNode, q, level)]
            for endNode in graph.nodes() if endNode != 0
            for q in range(1, Q + 1, 1)
            for level in range(L + 1, 0, -1)
        )
        , GRB.MINIMIZE,
    )

    if relax == True:

        for key in arcsVisitedByDrone.keys():
            arcsVisitedByDrone[key].vtype = GRB.CONTINUOUS
        model.write(
            'models\\' + "relaxed_MTRPD_LBD_cL_SP_[Instance = {0}, Drones = {1}, Allowed Nr. of Drone Trips = {2}, Flight Range = {3}].lp".format(
                graph, K, Q, E))
        return model.relax(), constraints, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime
    else:
        model.write(
            'models\\' + "MTRPD_LBD_cL_SP_[Instance = {0}, Drones = {1}, Allowed Nr. of Drone Trips = {2}, Flight Range = {3}].lp".format(
                graph, K, Q, E))
        return model, constraints, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime


def get_arc_list_of_csv(path):
    df = pd.read_csv(path)
    result = [
        (df['Node1'][ind], df['Node2'][ind], df['traveling time'][ind])
        for ind in df.index
    ]
    return result


def plot_graph(G, dictNodePos, *args):
    if len(args) != 0:
        nx.draw(G, with_labels=True, pos=dictNodePos, node_color=[color for color in args[0].values()])
    else:
        nx.draw(G, with_labels=True, pos=dictNodePos)
    arcLabels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos=dictNodePos, edge_labels=arcLabels)
    plt.show()


# return the list of arcs of the form (i, j, distance(i,j))
def read_instance_file(fileName):
    with open(fileName, 'r') as instance:
        # linesNumbers = [8, 10] # line 8 is the depot and starting from line 10 we have the customer nodes
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


def solve_MP_model(model,
                   nodesVisitedByTruck,
                   arcsVisitedByTruck,
                   arcsVisitedByDrone,
                   truckArrivingTime,
                   droneArrivingTime,
                   efficientSolvingGurobi,
                   graph,
                   droneSpeed,
                   t0): #t0 The start time of solving the whole problem --> The start time in the function solve-LBD-cl
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
    model.setParam('TimeLimit', 60 * 60)

    # set log file
    #model.setParam('LogFile', "Logs\\" + str(graph) + '.txt')

    def cb(model, where):
        if where == GRB.Callback.MIP:
            if time.time() - t0 > 3600:
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
    chosenArcsVisitedByTruck = {}  # (startNode, endNode, level): 1
    chosenArcsVisitedByDrone = {}  # (startNode, endNode, drone, level): 1
    resultTruckArrivingTime = {}  # level: value > 0
    resultDroneArrivingTime1 = {}  # (endNode,level): value > 0
    resultDroneArrivingTime2 = {}  # (launchNode, drone, droneTrip): value > 0
    resultDroneArrivingTime3 = {}  # (endNode,level): value > 0
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
    for (endNode, level) in droneArrivingTime[0].keys():
        if droneArrivingTime[0][(endNode, level)].X > 0:
            resultDroneArrivingTime1[(endNode, level)] = droneArrivingTime[0][(endNode, level)].X
    for (launchNode, drone, droneTrip) in droneArrivingTime[1].keys():
        if droneArrivingTime[1][(launchNode, drone, droneTrip)].X > 0:
            resultDroneArrivingTime2[(launchNode, drone, droneTrip)] = droneArrivingTime[1][
                (launchNode, drone, droneTrip)].X
    for (endNode, level) in droneArrivingTime[2].keys():
        if droneArrivingTime[2][(endNode, level)].X > 0:
            resultDroneArrivingTime3[(endNode, level)] = droneArrivingTime[2][(endNode, level)].X

    result = {}
    #result['graph'] = H
    result['nodesColors'] = colors
    result['truckNodes'] = chosenNodesVisitedByTruck
    result['truckArcs'] = chosenArcsVisitedByTruck
    result['droneArcs'] = chosenArcsVisitedByDrone
    result['truckArrivingTimes'] = resultTruckArrivingTime
    result['droneArrivingTimes'] = resultDroneArrivingTime
    result['objectiveValue'] = model.objVal
    # result['optGap'] = model.MIPGap
    # result['cpu'] = t00 - t0
    return result


def solve_SP_model(model,
                   chosenNodesVisitedByTruck,
                   chosenArcsVisitedByTruck,
                   arcsVisitedByDrone,
                   truckArrivingTime,
                   droneArrivingTime,
                   efficientSolvingGurobi,
                   graph,
                   droneSpeed,
                   t0): #t0 The start time of solving the whole problem --> The start time in the function solve-LBD-cl
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
    model.setParam('TimeLimit', 60 * 60)
    # set log file
    #model.setParam('LogFile', "Logs\\" + str(graph) + '.txt')

    def cb(model, where):
        if where == GRB.Callback.MIP:
            if time.time() - t0 > 3600:
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
    #result['graph'] = H
    result['nodesColors'] = colors
    result['droneArcs'] = chosenArcsVisitedByDrone
    result['truckArrivingTimes'] = resultTruckArrivingTime
    result['droneArrivingTimes'] = resultDroneArrivingTime
    result['objectiveValue'] = model.objVal

    return result


def solve_model_LBD(instancePath, allowedNumberOfTruckCustomers, droneSpeed, numberOfDrones, numberOfTrips, largeNumber,
                    efficientSolvingGurobi, relax, lowestUpperBound, t0, coveragePercentage): #t0 The start time of solving the whole problem --> The start time in the function solve-LBD-cl
    instanceLocations, instanceArcs, graph = read_instance_file(instancePath)
    travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
    travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}
    #flightEndurance = np.ceil(2 * max(travelingTimeDrone.values()))
    flightEndurance = flight_endurance(travelingTimeDrone, graph.number_of_nodes()-1, coveragePercentage)

    t1 = time.time()
    t3 = time.time()

    solutionGraphplot = nx.DiGraph()
    nodeColorsplot = {}

    status = ''  # GRB.INFEASIBLE : 3, GRB.OPTIMAL : 2, GRB.TIME_LIMIT : 9
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

    modelMP, constraintsMP, objectiveValueMP, nodesVisitedByTruckMP, arcsVisitedByTruckMP, arcsVisitedByDroneMP, truckArrivingTimeMP, droneArrivingTimeMP = \
        create_MTRPD_MP(graph, allowedNumberOfTruckCustomers, numberOfDrones, numberOfTrips, flightEndurance, droneSpeed, relax)

    L = - 1
    U = GRB.INFINITY
    it = 1
    flag = True

    while flag:
        if round(U, 5) <= round(L, 5) or L > lowestUpperBound:
            #plot_graph(solutionGraphplot, instanceLocations, nodeColorsplot)
            # plotName = "figures\\LBD_"+ instancePath[10:(len(instancePath) - 4)] + str(time.time())+'.png'
            # plt.savefig(plotName)

            result = {}
            if L > lowestUpperBound:
                result['optGap'] = 'no better solution found'
            else:
                result['optGap'] = round(((U - L) / U) * 100, 4)

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
            for currentIt in range(it-1, 0, -1):
                if abs(bounds[currentIt]['SPObjectiveValue'] - U) <= 0.0001:
                    result['searchedSolutionIteration'] = currentIt
                    result['objectiveValue'] = U
                    result['droneShare'] = round(
                        len(result['droneArcsSP'][currentIt])
                        /
                        (graph.number_of_nodes()-1)
                        , 4)
                    return result

        # STEP 1 solve MP
        solutionMP = solve_MP_model(modelMP,
                                    nodesVisitedByTruckMP,
                                    arcsVisitedByTruckMP,
                                    arcsVisitedByDroneMP,
                                    truckArrivingTimeMP,
                                    droneArrivingTimeMP,
                                    efficientSolvingGurobi,
                                    graph,
                                    droneSpeed,
                                    t0)
        if not solutionMP:
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
            result['optGap'] = round(optGap, 4)
            result['optCuts'] = optCuts
            return result

        else:

            # Time needed to solve the Master Problem:
            if it == 1:
                t2 = time.time()
                cpuMP[it] = int(t2 - t1)
            else:
                t2 = time.time()
                cpuMP[it] = int(t2 - t3)

            if modelMP.status == GRB.TIME_LIMIT or modelMP.status == GRB.INTERRUPTED:
                status = GRB.TIME_LIMIT
                cpuSP[it] = 0
                result = {}
                try:
                    result['optGap'] = round(((U - solutionMP['objectiveValue']) / U) * 100, 4)
                    result['objectiveValue'] = U
                    result['droneShare'] = round(
                        len(result['droneArcsSP'][it])
                        /
                        (graph.number_of_nodes() - 1)
                        , 4)
                except:
                    result['optGap'] = round(((U - L) / U) * 100, 4)
                    

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

                resultNodesTruck[it] = dict(
                    sorted({key: value for (key, value) in solutionMP['truckNodes'].items() if value > 0}.items(),
                           key=lambda t: t[0][1], reverse=True))
                resultArcsTruck[it] = dict(
                    sorted({key: value for (key, value) in solutionMP['truckArcs'].items() if value > 0}.items(),
                           key=lambda t: t[0][2], reverse=True))
                resultArcsDroneMP[it] = dict(
                    sorted({key: value for (key, value) in solutionMP['droneArcs'].items() if value > 0}.items(),
                           key=lambda t: t[0][3], reverse=True))
                resultArrivingTimeTruckMP[it] = dict(sorted(
                    {key: value for (key, value) in solutionMP['truckArrivingTimes'].items() if value > 0}.items(),
                    key=lambda t: t[0], reverse=True))
                resultArrivingTimeDroneMP1 = dict(sorted(
                    {key: value for (key, value) in solutionMP['droneArrivingTimes'][0].items() if value > 0}.items(),
                    key=lambda t: t[0][1], reverse=True))
                resultArrivingTimeDroneMP2 = dict(sorted(
                    {key: value for (key, value) in solutionMP['droneArrivingTimes'][1].items() if value > 0}.items(),
                    key=lambda t: t[0][2]))
                resultArrivingTimeDroneMP3 = dict(sorted(
                    {key: value for (key, value) in solutionMP['droneArrivingTimes'][2].items() if value > 0}.items(),
                    key=lambda t: t[0][1], reverse=True))
                resultArrivingTimeDroneMP[it] = {}
                resultArrivingTimeDroneMP[it]['ArrivingTimeDroneMP1'] = resultArrivingTimeDroneMP1
                resultArrivingTimeDroneMP[it]['ArrivingTimeDroneMP2'] = resultArrivingTimeDroneMP2
                resultArrivingTimeDroneMP[it]['ArrivingTimeDroneMP3'] = resultArrivingTimeDroneMP3
                # plot_graph(solutionGraphMP, instanceLocations, nodeColorsMP)

                # set the lower bound
                L = solutionMP['objectiveValue']
                bounds[it] = {}
                bounds[it]['MPObjectiveValue'] = L

                # STEP 2 create and solve SP
                modelSP, constraintsSP, arcsVisitedByDroneSP, truckArrivingTimeSP, droneArrivingTimeSP = \
                    create_MTRPD_SP(graph, allowedNumberOfTruckCustomers, numberOfDrones, numberOfTrips, largeNumber,
                                    flightEndurance, droneSpeed, solutionMP['truckNodes'], solutionMP['truckArcs'],
                                    relax)
                solutionSP = solve_SP_model(modelSP,
                                            solutionMP['truckNodes'], solutionMP['truckArcs'],
                                            arcsVisitedByDroneSP,
                                            truckArrivingTimeSP,
                                            droneArrivingTimeSP,
                                            efficientSolvingGurobi,
                                            graph,
                                            droneSpeed,
                                            t0)
                if modelSP.status == GRB.UNBOUNDED:
                    # add a feasibility cut to the MP
                    constraintsMP['(feasibilityCut)_{}'.format(it)] = modelMP.addConstr(
                        sum(
                            (1 - nodesVisitedByTruckMP[(endNode, level)])
                            for (endNode, level) in solutionMP['truckNodes'].keys() if
                            solutionMP['truckNodes'][(endNode, level)] > 0.99
                        )
                        +
                        sum(
                            nodesVisitedByTruckMP[(endNode, level)]
                            for (endNode, level) in solutionMP['truckNodes'].keys() if
                            solutionMP['truckNodes'][(endNode, level)] == 0
                        )
                        >= 1
                        ,
                        name='(feasibilityCut)_{}'.format(it)
                    )
                    modelMP.update()
                    # modelMP.reset()
                    modelMP.write(
                        'models\\' + "MTRPD_LBD_cL_MP_[Instance = {0}, Drones = {1}, Allowed Nr. of Drone Trips = {2}, Flight Range = {3}].lp".format(
                            graph, numberOfDrones, numberOfTrips, flightEndurance))
                    it += 1
                else:
                    # Time needed to solve the Sub Problem:
                    t3 = time.time()
                    cpuSP[it] = int(t3 - t2)
                    if modelSP.status == GRB.TIME_LIMIT or modelSP.status == GRB.INTERRUPTED:
                        # time limit within one iteration reached
                        status = GRB.TIME_LIMIT
                        #lowestUB = solutionSP['objectiveValue']
                        #U = min(U, lowestUB)
                        bounds[it]['SPObjectiveValue'] = GRB.INFINITY
                        bounds[it]['globalUpperBound'] = GRB.INFINITY
                        bounds[it]['optGap'] = ((U - L) / U) * 100
                        optGap = bounds[it]['optGap']

                        result = {}
                        try:
                            for currentIt in range(it, 0, -1):
                                if abs(bounds[currentIt]['SPObjectiveValue'] - U) <= 0.0001:
                                    result['droneShare'] = round(
                                        len(result['droneArcsSP'][currentIt])
                                        /
                                        (graph.number_of_nodes() - 1)
                                        , 4)
                        except:
                            result['droneShare'] = 0

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
                        result['optGap'] = round(optGap, 4)
                        result['optCuts'] = optCuts
                        result['objectiveValue'] = U

                        return result
                    else:
                        # SP solved to optimality within the time limit
                        status = GRB.OPTIMAL

                        resultArcsDroneSP[it] = dict(sorted(
                            {key: value for (key, value) in solutionSP['droneArcs'].items() if value > 0}.items(),
                            key=lambda t: t[0][4], reverse=True))
                        resultArrivingTimeTruckSP[it] = dict(sorted(
                            {key: value for (key, value) in solutionSP['truckArrivingTimes'].items() if
                             value > 0}.items(),
                            key=lambda t: t[0], reverse=True))
                        resultArrivingTimeDroneSP[it] = dict(sorted(
                            {key: value for (key, value) in solutionSP['droneArrivingTimes'].items() if
                             value > 0}.items(),
                            key=lambda t: t[0][2], reverse=True))
                        # set the plot
                        #solutionGraphplot = solutionSP['graph']
                        #nodeColorsplot = solutionSP['nodesColors']
                        # plot_graph(solutionSP['graph'], instanceLocations, olutionSP['nodesColors'])
                        lowestUB = solutionSP['objectiveValue']
                        U = min(U, lowestUB)
                        bounds[it]['SPObjectiveValue'] = lowestUB
                        bounds[it]['globalUpperBound'] = U
                        bounds[it]['optGap'] = ((U - L) / U) * 100

                        # add the optimality Cut
                        constraintsMP['(optimalityCut)_{}'.format(it)] = modelMP.addConstr(
                            objectiveValueMP['eta']
                            >=
                            lowestUB
                            * (
                                    1
                                    -
                                    sum(
                                        (1 - nodesVisitedByTruckMP[(endNode, level)])
                                        for (endNode, level) in solutionMP['truckNodes'].keys() if
                                        solutionMP['truckNodes'][(endNode, level)] > 0.99
                                    )
                                    -
                                    sum(
                                        nodesVisitedByTruckMP[(endNode, level)]
                                        for (endNode, level) in solutionMP['truckNodes'].keys() if
                                        solutionMP['truckNodes'][(endNode, level)] < 0.01
                                    )
                            )
                            ,
                            name='(optimalityCut)_{}'.format(it)
                        )
                        modelMP.update()
                        #modelMP.reset()
                        modelMP.write(
                            'models\\' + "MTRPD_LBD_cL_MP_[Instance = {0}, Drones = {1}, Allowed Nr. of Drone Trips = {2}, Flight Range = {3}].lp".format(
                                graph, numberOfDrones, numberOfTrips, flightEndurance))
                        t3 = time.time()
                        if t3 - t0 > 3600:
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
                            result['optGap'] = round(((U - L) / U) * 100, 4)
                            result['optCuts'] = optCuts
                            result['objectiveValue'] = U
                            result['droneShare'] = round(
                                len(result['droneArcsSP'][it])
                                /
                                (graph.number_of_nodes() - 1)
                                , 4)
                            return result
                        it += 1



def solving_model_LBD_efficient(instancePath, droneSpeed, numberOfDrones, numberOfTrips,
                                largeNumber, efficientSolvingGurobi, relax, coveragePercentage):
    # get the instance locations (dict -> node:(x,y)); instance arcs (list -> [i, j, dist(i,j)]); and the graph (networkx object)
    instanceLocations, instanceArcs, graph = read_instance_file(instancePath)
    # get the traveling times of the truck (dict -> (i, j) : dist(i,j))
    #travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
    # get the traveling times of a drone (dict -> (i, j) : dist(i,j))
    #travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}
    # get the flight endurance. current flight endurance is chosen to be infinity
    #flightEndurance = np.ceil(2 * max(travelingTimeDrone.values()))

    # start number of levels. l_0, which is the number of truck customers
    numberOfLevels = int((graph.number_of_nodes() - 1) / (numberOfDrones*numberOfTrips+ 1))

    #initialize lowestUpperBound
    lowestUpperBound = GRB.INFINITY
    # start time
    t0 = time.time()

    # track the solutions. (dict -> level: solutiondict)
    allSolutions = {}
    while True:
        currentSolution = solve_model_LBD(instancePath, numberOfLevels, droneSpeed, numberOfDrones, numberOfTrips,
                                          largeNumber, efficientSolvingGurobi, relax, lowestUpperBound, t0, coveragePercentage)
        # the current problem is infeasible
        if currentSolution['status'] == GRB.INFEASIBLE:
            if numberOfLevels == graph.number_of_nodes() - 1:
                currentSolution['status'] = GRB.TIME_LIMIT
                tFinal = time.time()
                allSolutions['cpu-totalTime-allLevels'] = tFinal - t0
                return currentSolution, allSolutions
            numberOfLevels = numberOfLevels + 1
        else:
            allSolutions[0] = currentSolution
            currentSolution['numberOfLevels'] = numberOfLevels
            # the current problem has reached the time limit
            if currentSolution['status'] == GRB.INTERRUPTED or currentSolution['status'] == GRB.TIME_LIMIT:
                tFinal = time.time()
                allSolutions['cpu-totalTime-allLevels'] = tFinal - t0
                return currentSolution, allSolutions

            # the current problem is solved
            else:
                break

    searchedIteration = currentSolution['searchedSolutionIteration']
    currentSolutionObjVal = currentSolution['bounds'][searchedIteration]['SPObjectiveValue']
    truckCustomers = currentSolution['truckArcs'][searchedIteration]
    currentNumberOfTruckCustomers = len(truckCustomers)
    # update lowest upper bound
    lowestUpperBound = currentSolutionObjVal
    #initialize the solution to be returned
    nextLevelResultToReturn = currentSolution
    it = 1

    while it < graph.number_of_nodes() - currentNumberOfTruckCustomers:
        # update the number of levels: l_(i+1) = l_i + 1
        numberOfLevels = numberOfLevels + 1
        # solve the problem P_(i+1)
        nextLevelResult = solve_model_LBD(instancePath, numberOfLevels, droneSpeed, numberOfDrones,
                                                  numberOfTrips,
                                                  largeNumber, efficientSolvingGurobi, relax, lowestUpperBound, t0, coveragePercentage)

        # add the number of levels = truck customers to the solution
        nextLevelResult['numberOfLevels'] = numberOfLevels
        # add the solution to overall results
        allSolutions[it] = nextLevelResult
        if nextLevelResult['status'] == GRB.TIME_LIMIT or nextLevelResult['status'] == GRB.INTERRUPTED or nextLevelResult['status'] == GRB.INFEASIBLE:
            currentSolution['status'] = GRB.TIME_LIMIT
            tFinal = time.time()
            allSolutions['cpu-totalTime-allLevels'] = tFinal - t0
            return nextLevelResultToReturn, allSolutions
        # get the searched iteration
        searchedIteration = nextLevelResult['searchedSolutionIteration']
        # get the objective value of the solution
        nextLevelResultObjVal = nextLevelResult['bounds'][searchedIteration]['SPObjectiveValue']

        #update the lowest upper bound if a better solution was found
        if nextLevelResultObjVal < lowestUpperBound:
            nextLevelResultToReturn = nextLevelResult
            lowestUpperBound = nextLevelResultObjVal
        it = it + 1
    tFinal = time.time()
    allSolutions['cpu-totalTime-allLevels'] = tFinal - t0
    return nextLevelResultToReturn, allSolutions

#coverageRange: number between 0 and 1
def flight_endurance(timeTravellingDrone, numberOfCustomers, coverageRange):
    R = 0.5
    totalNumberOfFeasibleFlights = 0
    while (totalNumberOfFeasibleFlights / (numberOfCustomers*(numberOfCustomers+1))) < coverageRange:
        totalNumberOfFeasibleFlights = 0
        R += 0.5
        travellingTimes = timeTravellingDrone.values()
        for travellingTime in travellingTimes:
            if 2*travellingTime <= R:
                totalNumberOfFeasibleFlights += 1
    return R

def main():
    # allnodes: Truck and drone nodes
    # todo: which drones could not be reach by any drone from any node (preprocessing)
    # todo: set of nodes that could be reach by a drone from a node to minimize the search space
    # nodeList_2customers = [i for i in range(0, 3, 1)]
    # nodeList_3customers = [i for i in range(0, 4, 1)]
    # nodeList_4customers = [i for i in range(0, 5, 1)]
    # nodeList_6customers = [i for i in range(0, 7, 1)]

    # positions of the nodes
    # position list -> [(longitude, latitude)]
    # posList_2customers = [(1, 1), (3, 1), (3, 2)]  # 0, 1, 2
    # posList_3customers = [(1, 1), (3, 1), (3.5, 2), (3.5, 0)]  # 0, 1, 2, 3
    # posList_4customers = [(1, 1), (3, 1), (3, 1.5), (3, 0.5), (6, 1)]  # 0, 1, 2, 3, 4
    # posList_6customers = [(1, 1), (3, 1), (3, 1.5), (3, 0.5), (6, 1), (8, 1.25), (8, 0.75)]  # 0, 1, 2, 3, 4, 5, 6

    # position dict -> node : (longitude, latitude)
    # nodePos_2customers = {node: pos for (node, pos) in zip(nodeList_2customers, posList_2customers)}
    # nodePos_3customers = {node: pos for (node, pos) in zip(nodeList_3customers, posList_3customers)}
    # nodePos_4customers = {node: pos for (node, pos) in zip(nodeList_4customers, posList_4customers)}
    # nodePos_6customers = {node: pos for (node, pos) in zip(nodeList_6customers, posList_6customers)}

    # arcs (start, end, traveling time)

    # we add here r before the path to avoid having problems because of the slash, r is giving a raw stringraph. This means that for example
    # \U will not be interpreted as something special
    # arcList_2customers = get_arc_list_of_csv(r"Instances\MTRPD-2-customers.csv")
    # arcList_3customers = get_arc_list_of_csv(r"Instances\MTRPD-3-customers.csv")
    # arcList_4customers = get_arc_list_of_csv(r"Instances\MTRPD-4-customers.csv")
    # arcList_6customers = get_arc_list_of_csv(r"Instances\MTRPD-6-customers.csv")

    # instance7NodesUniform = r"Instances\uniform-1-n7.txt"
    # instance7NodesUniformLocations, instance7NodesUniformArcs, graph7 = read_instance_file(instance7NodesUniform)
    # instance8NodesUniform = r'Instances\uniform-1-n8.txt'
    # instance8NodesUniformLocations, instance8NodesUniformArcs, graph8 = read_instance_file(instance8NodesUniform)
    # instance9NodesUniform = r'Instances\uniform-1-n9.txt'
    # instance9NodesUniformLocations, instance9NodesUniformArcs, graph9 = read_instance_file(instance9NodesUniform)
    # instance10NodesUniform = r'Instances\uniform-1-n10.txt'
    # instance10NodesUniformLocations, instance10NodesUniformArcs, graph10u = read_instance_file(instance10NodesUniform)
    # instance10NodesCentroidUniform = r'Instances\centroid-uniform-1-n11.txt'
    # instance10NodesCentroidUniformLocations, instance10NodesCentroidUniformArcs, graph10cu = read_instance_file(
    #     instance10NodesCentroidUniform)
    # instance11NodesUniform = r'Instances\uniform-1-n11.txt'
    # instance11NodesUniformLocations, instance11NodesUniformArcs, graph11u = read_instance_file(instance11NodesUniform)

    # not solvable:
    # instance20NodesUniform = r'Instances\uniform-1-n20.txt'
    # instance20NodesUniformLocations, instance20NodesUniformArcs, graph20u = read_instance_file(instance20NodesUniform)

    # Plot the graph
    # plot_graph(G10CentroidUniform, instance10NodesCentroidUniformLocations)

    # all variables:
    # instance = r"Instances\uniform-centroid-0-n8.txt"
    # instanceLocations, instanceArcs, graph = read_instance_file(instance)
    droneSpeed = 2  # {2}
    numberOfDrones1 = 1  # {1, 2, 3}
    numberOfTrips1 = 1 # per node per launch customer {1, 2, max = ceiling(numberOfCustomers/numberOfDrones)} # - 1 since at least one customer is visited by the truck
    numberOfDrones2 = 2  # {1, 2, 3}
    numberOfTrips2 = 2  # per node per launch customer {1, 2, max = ceiling(numberOfCustomers/numberOfDrones)} # - 1 since at least one customer is visited by the truck

    largeNumber = 1000000

    # flightEndurance = np.ceil(2 * max(nx.get_edge_attributes(graph, 'weight').values()))
    efficientSolvingGurobi = True
    relax = False

    # instance:runtime
    results = {}
    resultsDetailed = {}


    instancePath = r"Instances\clustered-origin-317-n11.txt"
    instance = instancePath[10:(len(instancePath) - 4)]
    print('-------> current Instance: ', instance)
    result = solving_model_LBD_efficient(instancePath, droneSpeed, 1, 3,
                                         largeNumber, efficientSolvingGurobi, relax, 1)
    results[instance] = result[0]
    resultsDetailed[instance] = result[1]
    #uniform centroid - 16 nodes
    for i in range(10):
        instancePath = r"Instances\uniform-centroid-6" + str(i) + "-n16.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        coveragePercentage = 0.25
        result = solving_model_LBD_efficient(instancePath, droneSpeed, numberOfDrones1, numberOfTrips1,
                                                        largeNumber, efficientSolvingGurobi, relax, coveragePercentage)
        results[instance] = result[0]
        resultsDetailed[instance] = result[1]
        fileName = 'n16-C25percent-DroneSpeed_=_{0},_NrDrones_=_{1},_DroneTrips_=_{2}\\MTRPD_LBD_cL_uc' \
                   '[DroneSpeed_=_{0},_NrDrones_=_{1},_DroneTrips_=_{2},_timeStamp_=_{3},_instance_={4}]'.format(
            droneSpeed, numberOfDrones1, numberOfTrips1, round(time.time()/10**8, 2), i)
        # export_import_Results.exportCPUToExcel_LBD_cL(resultsDetailed, results, columnsOverview, columnsPerInstance, fileName)
        export_import_Results.exportAllResultsToTextFile(resultsDetailed, fileName + '_detailed')
        export_import_Results.exportAllResultsToTextFile(results, fileName + '_optimalSolutionsOnly')

    for i in range(10):
        instancePath = r"Instances\uniform-centroid-6" + str(i) + "-n16.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        coveragePercentage = 0.25
        result = solving_model_LBD_efficient(instancePath, droneSpeed, numberOfDrones2, numberOfTrips2,
                                                        largeNumber, efficientSolvingGurobi, relax, coveragePercentage)
        results[instance] = result[0]
        resultsDetailed[instance] = result[1]
        fileName = 'n16-C25percent-DroneSpeed_=_{0},_NrDrones_=_{1},_DroneTrips_=_{2}\\MTRPD_LBD_cL_uc' \
                   '[DroneSpeed_=_{0},_NrDrones_=_{1},_DroneTrips_=_{2},_timeStamp_=_{3},_instance_={4}]'.format(
            droneSpeed, numberOfDrones2, numberOfTrips2, round(time.time()/10**8, 2), i)
        # export_import_Results.exportCPUToExcel_LBD_cL(resultsDetailed, results, columnsOverview, columnsPerInstance, fileName)
        export_import_Results.exportAllResultsToTextFile(resultsDetailed, fileName + '_detailed')
        export_import_Results.exportAllResultsToTextFile(results, fileName + '_optimalSolutionsOnly')

    #uniform centroid - 11 nodes
    for i in range(10):
        instancePath = r"Instances\uniform-centroid-1" + str(i) + "-n11.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        result = solving_model_LBD_efficient(instancePath, droneSpeed, numberOfDrones, numberOfTrips,
                                                        largeNumber, efficientSolvingGurobi, relax)
        results[instance] = result[0]
        resultsDetailed[instance] = result[1]
        fileName = 'DroneSpeed_=_2,_NrDrones_=_3,_DroneTrips_=_3\\MTRPD_LBD_cL_uc[DroneSpeed_=_{0},_NrDrones_=_{1},_DroneTrips_=_{2},_timeStamp_=_{3},_instance_={4}]'.format(
            droneSpeed, numberOfDrones, numberOfTrips, round(time.time()/10**8, 2), i)
        # export_import_Results.exportCPUToExcel_LBD_cL(resultsDetailed, results, columnsOverview, columnsPerInstance, fileName)
        export_import_Results.exportAllResultsToTextFile(resultsDetailed, fileName + '_detailed')
        export_import_Results.exportAllResultsToTextFile(results, fileName + '_optimalSolutionsOnly')


    #uniform origin - 11 nodes
    for i in range(10):
        instancePath = r"Instances\uniform-origin-11" + str(i) + "-n11.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        result = solving_model_LBD_efficient(instancePath, droneSpeed, numberOfDrones, numberOfTrips,
                                                        largeNumber, efficientSolvingGurobi, relax)
        results[instance] = result[0]
        resultsDetailed[instance] = result[1]
        fileName = 'DroneSpeed_=_2,_NrDrones_=_3,_DroneTrips_=_3\\MTRPD_LBD_cL_uo[DroneSpeed_=_{0},_NrDrones_=_{1},_DroneTrips_=_{2},_timeStamp_=_{3},_instance_={4}]'.format(
            droneSpeed, numberOfDrones, numberOfTrips, round(time.time()/10**8, 2), i)
        # export_import_Results.exportCPUToExcel_LBD_cL(resultsDetailed, results, columnsOverview, columnsPerInstance, fileName)
        export_import_Results.exportAllResultsToTextFile(resultsDetailed, fileName + '_detailed')
        export_import_Results.exportAllResultsToTextFile(results, fileName + '_optimalSolutionsOnly')

    # clustered centroid - 11 nodes
    for i in range(10):
        instancePath = r"Instances\clustered-centroid-21" + str(i) + "-n11.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        result = solving_model_LBD_efficient(instancePath, droneSpeed, numberOfDrones, numberOfTrips,
                                             largeNumber, efficientSolvingGurobi, relax)
        results[instance] = result[0]
        resultsDetailed[instance] = result[1]
        fileName = 'DroneSpeed_=_2,_NrDrones_=_3,_DroneTrips_=_3\\MTRPD_LBD_cL_cc[DroneSpeed_=_{0},_NrDrones_=_{1},_DroneTrips_=_{2},_timeStamp_=_{3},_instance_={4}]'.format(
            droneSpeed, numberOfDrones, numberOfTrips, round(time.time()/10**8, 2), i)
        # export_import_Results.exportCPUToExcel_LBD_cL(resultsDetailed, results, columnsOverview, columnsPerInstance, fileName)
        export_import_Results.exportAllResultsToTextFile(resultsDetailed, fileName + '_detailed')
        export_import_Results.exportAllResultsToTextFile(results, fileName + '_optimalSolutionsOnly')


    #clustered origin - 11 nodes
    for i in range(10):
        instancePath = r"Instances\clustered-origin-31" + str(i) + "-n11.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        result = solving_model_LBD_efficient(instancePath, droneSpeed, numberOfDrones, numberOfTrips,
                                             largeNumber, efficientSolvingGurobi, relax)
        results[instance] = result[0]
        resultsDetailed[instance] = result[1]
        fileName = 'DroneSpeed_=_2,_NrDrones_=_3,_DroneTrips_=_3\\MTRPD_LBD_cL_co[DroneSpeed_=_{0},_NrDrones_=_{1},_DroneTrips_=_{2},_timeStamp_=_{3},_instance_={4}]'.format(
            droneSpeed, numberOfDrones, numberOfTrips, round(time.time()/10**8, 2), i)
        # export_import_Results.exportCPUToExcel_LBD_cL(resultsDetailed, results, columnsOverview, columnsPerInstance, fileName)
        export_import_Results.exportAllResultsToTextFile(resultsDetailed, fileName + '_detailed')
        export_import_Results.exportAllResultsToTextFile(results, fileName + '_optimalSolutionsOnly')

    columnsOverview = ['Instance', 'CPU', 'GAP', 'Objective Value', 'Drones share']
    columnsPerInstance = ['number of levels', 'CPU', 'GAP', 'Objective Value']
    fileName = 'MTRPD_LBD_cL[DroneSpeed_=_{0},_NrDrones_=_{1},_DroneTrips_=_{2},_timeStamp_=_{3}]'.format(
        droneSpeed, numberOfDrones, numberOfTrips, round(time.time()/10**8, 2))
    #export_import_Results.exportCPUToExcel_LBD_cL(resultsDetailed, results, columnsOverview, columnsPerInstance, fileName)
    export_import_Results.exportAllResultsToTextFile(resultsDetailed, fileName+'_detailed')
    export_import_Results.exportAllResultsToTextFile(results, fileName+'_optimalSolutionsOnly')

    #return results



if __name__ == '__main__':
    main()