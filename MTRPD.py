#gurobi version 10.0.0
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
import time

#function to create the graph
import export_import_Results


def create_graph(nodeList, arcList):
    # Create graph
    graph = nx.DiGraph()
    # add nodes
    graph.add_nodes_from(nodeList)
    # add arcs
    graph.add_weighted_edges_from(arcList)
    return graph

#G -> Graph;
# K -> Number of available drones;
# Q -> Number of possible trips;
# M -> large number;
# E -> flight endurance of the drones;
# S -> Drone Speed
def create_MTRPD(graph, L, K, Q, M, E, S, relax):


    #Initialize the model
    model = gp.Model()
    # in case we don't need output
    #model.setParam('OutputFlag', False)

    #number of Levels / customers to be visited in the backward order
    #L = graph.number_of_nodes() - 1

    #get the traveling times (dict) --> (arc) : weight
    travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
    travelingTimeDrone = {arc: int(time / S) for (arc, time) in travelingTimeTruck.items()}
    #nodes - L dict -> (node, level): Gurobi variables
    nodesVisitedByTruck = {}

    #arcs dict -> arc - level : Gurobi varialbes
    arcsVisitedByTruck = {}
    #arcs dict -> arc - level - drone - trip : Gurobi varialbes
    arcsVisitedByDrone = {}

    #Arriving time of Truck in the level l
    #L dict -> level : Gurobi variable
    truckArrivingTime = {}

    #Arriving time of a drone at a customer j in the level l in its qth trip
    #L-customers'-trip dict -> L-customers'-trip : Gurobi variable
    droneArrivingTime = {}
    


    #Gurobi Variables:
    # binary node variable to assign a customer to the truck at level l.
    # x_i^l == 1 if node i outOf T is visited in the backward position l
    for node in graph.nodes():
        for level in range(L, 0, -1):
            nodesVisitedByTruck[(node, level)] = \
                model.addVar(
                    vtype= GRB.BINARY,
                    lb = 0,
                    ub = 1,
                    name = "x_{0}^{1}".format(node, level)
                )
    for arc in graph.edges():
        #In the first level (L) we have only the arcs from the depot to all other customers
        if arc[0] == 0:
            for level in range(L, 0, -1):
                # binary arc variable on the truck delivery route.
                # y_ij^l == 1 if e_(ij) is being used by the truck and exactly l customers need to be served by the truck after node i
                arcsVisitedByTruck[(arc[0], arc[1], level)] = \
                    model.addVar(
                        vtype= GRB.BINARY,
                        lb = 0,
                        ub = 1,
                        name = "y_{0}_{1}^{2}".format(arc[0], arc[1], level)
                    )

        else:
            for level in range(L-1, 0, -1):
                # binary arc variable on the truck delivery route.
                # y_ij^l == 1 if e_(ij) is being used by the truck and exactly l customers need to be served by the truck after node i
                arcsVisitedByTruck[(arc[0], arc[1], level)] = \
                    model.addVar(
                        vtype= GRB.BINARY,
                        lb = 0,
                        ub = 1,
                        name = "y_{0}_{1}^{2}".format(arc[0], arc[1], level)
                    )


    # binary arc variable on the drone delivery route.
    # f_ijkt^l == 1 if the truck launches drone k to serve node j outOf D in its tth trip, while waiting at node i in level l
    for arc in graph.edges(): #arc[0] != arc[1]
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
        #non-negative continuous variable representing the arrivel time at the lth last customer served by the truck
        #t^l
        truckArrivingTime[level] = model.addVar(
            vtype= GRB.CONTINUOUS,
            lb = 0,
            name = "t^{0}".format(level)
        )
        #non-negative continuous variable representing the arrivel time at node j outOf D served by a drone in its tth trip
        #while truck waiting at the lth last customer
        for node in graph.nodes():
            if node != 0:
                for droneTrip in range(1, Q+1, 1):
                    droneArrivingTime[(node, droneTrip, level)] = model.addVar(
                        vtype=GRB.CONTINUOUS,
                        lb = 0,
                        name="pi_{0}_{1}^{2}".format(node, droneTrip, level)
                    )

    #truck constraints:
    constraints = {i:[] for i in range(2,14)}
    #(2) the truck ends the route at a customer in the level 1 (the last level)
    #Definition: startNode is the starting node of an arc
    constraints[2].append(model.addConstr(
        sum(
            nodesVisitedByTruck[(startNode, 1)]
            for startNode in range(1, len(graph.nodes))
        )
        == 1,
        name='(2)_t'
    ))
    #(3) The truck leaves the depot to a node j in one of the levels l
    #Definition: endNode is the ending node of an arc
    constraints[3].append(model.addConstr(
        sum(
            arcsVisitedByTruck[(0, endNode, level)]
            for endNode in graph.nodes() if endNode != 0
            for level in range(L, 0, -1)
        )
        == 1,
        name='(3)_t'
    ))
    #(4) The connection between the nodes and arcs visited by the truck. - outbound
    # i.e. if a node i is visited by the truck in the level l+1 , then the truck needs to use one of the arcs in the level l to leave this node i
    for startNode in graph.nodes():
        for level in range(L - 1, 0, -1):
            constraints[4].append(model.addConstr(
                sum(
                    arcsVisitedByTruck[(startNode, endNode, level)]
                    for endNode in graph.nodes() if endNode != 0 and endNode != startNode
                )
                == nodesVisitedByTruck[(startNode, level + 1)],
                name='(4_[i = {0}, l = {1}])_t'.format(startNode,level)
            ))
    #(5) The connection between the nodes and arcs visited by the truck. - inbound
    for endNode in graph.nodes():
        if endNode != 0:
            for level in range(L-1, 0, -1):
                constraints[5].append(model.addConstr(arcsVisitedByTruck[(0, endNode, level)] +
                    sum(
                        arcsVisitedByTruck[(startNode, endNode, level)]
                        for startNode in graph.nodes() if startNode != 0 and startNode != endNode
                    )
                    == nodesVisitedByTruck[(endNode, level)],
                name='(5_[j = {0}, l = {1}])_t'.format(endNode, level)
                ))
    #(6) The connection between the node at the last level N and the arcs out of the depot to the nodes in this level
    for endNode in graph.nodes():
        if endNode != 0:
            constraints[6].append(model.addConstr(
                arcsVisitedByTruck[(0, endNode, L)] == nodesVisitedByTruck[(endNode, L)],
                name='(6_[j = {0}])_t'.format(endNode)
            ))
    #(7) all nodes/customers are visited either by the truck or the drone
    for endNode in graph.nodes():
        if endNode != 0:
            constraints[7].append(model.addConstr(
                sum(
                    nodesVisitedByTruck[(endNode, level)]
                    for level in range(L, 0, -1)
                )
                +
                sum(
                    arcsVisitedByDrone[(startNode, endNode, drone, droneTrip, level)]
                    for startNode in graph.nodes() if startNode != endNode
                    for drone in range(1, K+1, 1)
                    for droneTrip in range(1, Q+1, 1)
                    for level in range(L - 1, 0, -1)
                )
                +
                sum(
                    arcsVisitedByDrone[(0, endNode, drone, droneTrip, level)]
                    for drone in range(1, K+1, 1)
                    for droneTrip in range(1, Q+1, 1)
                    for level in range(L - 1, 0, -1)
                )
                == 1,
                name='(7_[i = {0}])'.format(endNode)
            ))
    #(9) Arriving time of the truck in the level l
    for level in range(L, 0, -1):
        for drone in range(1, K + 1, 1):

            #first step of the truck
            if level == L:
                constraints[9].append(model.addConstr(
                    # t^l
                    truckArrivingTime[level]
                    >=
                    sum(
                        travelingTimeTruck[(0, endNode)] * arcsVisitedByTruck[(0, endNode, level)]
                        for endNode in graph.nodes() if endNode != 0
                    ),
                    name='(9_[l = {0}])_t'.format(level)
                ))
            # starting from the second last step
            else:
                constraints[9].append(model.addConstr(
                    #t^l
                    truckArrivingTime[level]
                    >=
                    #previous traveling (and waiting time)
                    #we separate the following two sums because in level L there are only arcs out of the depot
                    #truck arcs out of the depot 0
                    sum(
                        travelingTimeTruck[(0, endNode)] * arcsVisitedByTruck[(0, endNode, previousLevel)]
                        for endNode in graph.nodes() if endNode != 0
                        for previousLevel in range(L, level, -1)
                    )
                    +
                    #arcs out of nodes other than the depot 0
                    sum(
                        travelingTimeTruck[(startNode, endNode)] * arcsVisitedByTruck[(startNode, endNode, previousLevel)]
                        for startNode in graph.nodes() if startNode != 0
                        for endNode in graph.nodes() if endNode != startNode and endNode != 0
                        for previousLevel in range(L - 1, level, -1) #since out of level L there are only arcs out of the depot
                    )
                    +
                    sum(
                        #todo: at level L it's unnecessary to consider the f_ijkq^L since those are automatically 0 if the truck is at level L
                        (travelingTimeDrone[(startNode, endNode)]+travelingTimeDrone[(endNode, startNode)]) * arcsVisitedByDrone[(startNode, endNode, drone, droneTrip, previousLevel)]
                        for startNode in graph.nodes() #if startNode != 0
                        for endNode in graph.nodes() if endNode != startNode and endNode != 0
                        for droneTrip in range(1, Q+1, 1)
                        for previousLevel in range(L, level, -1) #since out of level L there are only arcs out of the depot
                    )
                    #current traveling time of the truck
                    +
                    sum(
                        travelingTimeTruck[(startNode, endNode)] * arcsVisitedByTruck[(startNode, endNode, level)]
                        for startNode in graph.nodes()
                        for endNode in graph.nodes() if endNode != startNode and endNode != 0
                    ),
                    name='(9_[l = {0}])_t'.format(level)
                ))
    # Drone constraints
    #(8) drone trips constraints

    for level in range(L, 0, -1):
        # todo: at level L it is unnecassary to have those constraints since if x_i^3 == 1
        #  this means that all customers are visited by the truck, i.e. all f_ijkq^3 = 0
        for startNode in graph.nodes():
            for drone in range(1, K + 1, 1):
                constraints[8].append(model.addConstr(
                    sum(
                        arcsVisitedByDrone[(startNode, endNode, drone, droneTrip, level)]
                        for endNode in graph.nodes if endNode != startNode and endNode != 0
                        for droneTrip in range(1, Q+1, 1)
                    )
                    <= Q * nodesVisitedByTruck[(startNode, level)],
                    name='(8_[i = {0}, k = {1}, l = {2}])_d'.format(startNode, drone, level)
                ))

    #(10) Arriving time of a drone in its qth trip at customer j in the level l
    for endNode in graph.nodes():
        if endNode != 0:
            for drone in range(1, K+1, 1):
                for droneTrip in range(1, Q+1, 1):
                    for level in range(L, 0, -1):
                        if droneTrip == 1:
                            constraints[10].append(model.addConstr(
                                # pi_j_q^l
                                droneArrivingTime[(endNode, droneTrip, level)]
                                >=
                                truckArrivingTime[level]
                                +
                                #current traveling time to a customer j by the drone
                                sum(
                                    travelingTimeDrone[(launchNode, endNode)] * arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                                    for launchNode in graph.nodes() if launchNode != endNode
                                )
                                - M * (1 - sum(
                                    arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                                    for launchNode in graph.nodes() if launchNode != endNode
                                )),
                                name='(10_[j = {0}, k = {1}, q = 1, l = {2}])_d'.format(endNode, drone, level)
                            ))
                        #q > 1
                        else:
                            constraints[10].append(model.addConstr(
                                #pi_j_q^l
                                droneArrivingTime[(endNode, droneTrip, level)]
                                >=
                                truckArrivingTime[level]
                                +
                                #previous trips traveling time for a drone k
                                sum(
                                    (travelingTimeDrone[(launchNode, previousEndNode)] + travelingTimeDrone[(previousEndNode, launchNode)])
                                    * arcsVisitedByDrone[(launchNode, previousEndNode, drone, previousDroneTrip, level)]
                                    for launchNode in graph.nodes() if launchNode != endNode
                                    for previousEndNode in graph.nodes()
                                    if previousEndNode != 0 and previousEndNode != endNode and previousEndNode != launchNode
                                    for previousDroneTrip in range(1, droneTrip, 1)
                                )
                                +
                                # current traveling time for a drone k to a customer j
                                sum(
                                    travelingTimeDrone[(launchNode, endNode)] * arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                                    for launchNode in graph.nodes() if launchNode != endNode
                                )
                                - M * (1 - sum(
                                    arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                                    for launchNode in graph.nodes() if launchNode != endNode
                                )),
                                name='(10_[j = {0}, k = {1}, q = {2}, l = {3}])_d'.format(endNode, drone, droneTrip, level)
                            ))

    #(11) Each drone is visiting only one node per trip before coming back to the truck
    for launchNode in graph.nodes():
        for drone in range(1, K+1, 1):
            for droneTrip in range(1, Q+1, 1):
                constraints[11].append(model.addConstr(
                    sum(
                        arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                        for endNode in graph.nodes() if endNode != launchNode and endNode != 0
                        for level in range(L, 0, -1)
                    )
                    <= 1,
                    name='(11_[i = {0}, k = {1}, q = {2}])_d'.format(launchNode, drone, droneTrip)
                ))

    #(12) trip order is ensured
    if Q >= 2:
        for launchNode in graph.nodes():
            for drone in range(1, K+1, 1):
                for droneTrip in range(1, Q, 1):
                    for level in range(L, 0, -1):
                            constraints[12].append(model.addConstr(
                                sum(
                                    arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                                    for endNode in graph.nodes() if endNode != launchNode and endNode != 0
                                )
                                >=
                                sum(
                                    arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip + 1, level)]
                                    for endNode in graph.nodes() if endNode != launchNode and endNode != 0
                                ),
                                name='(12_[i = {0}, k = {1}, q = {2}, l = {3}])_d'.format(launchNode, drone, droneTrip, level)
                            ))

                    
    #(13) the flight endurance constraint of each drone
    for launchNode in graph.nodes():
        for endNode in graph.nodes():
            if endNode != launchNode and endNode != 0:
                for drone in range(1, K+1, 1):
                    for droneTrip in range(1, Q+1, 1):
                        for level in range(L, 0, -1): #level L is exculed since it's not considered. It's not possible in this model to start a drone in Level L, since if we send one drone, we would be in level L-1
                            constraints[13].append(model.addConstr(
                                (travelingTimeDrone[(launchNode, endNode)]+travelingTimeDrone[(endNode, launchNode)])
                                * arcsVisitedByDrone[(launchNode, endNode, drone, droneTrip, level)]
                                <=
                                E,
                                name='(13_[i = {0}, j = {1}, k = {2}, q = {3}, l = {4}])_d'.format(launchNode, endNode, drone, droneTrip, level)
                            ))

    #(test condition) the depot node shouldn't be visited by the truck at level 1
    model.addConstr(
        nodesVisitedByTruck[(0, 1)] == 0
    )
    #(1) set the objective: Sum of all traveling times of the truck and the drones
    model.setObjective(
        #truck
        sum(
            truckArrivingTime[level]
            for level in range(L, 0, -1)
        )
        +
        #drones
        sum(
            droneArrivingTime[(endNode, q, level)]
            for endNode in graph.nodes() if endNode != 0
            for q in range(1, Q+1, 1)
            for level in range(L, 0, -1)
        )
        , GRB.MINIMIZE,
    )
    #turn off the cuts, heuristics and presolving (but not quite working)
    #model.Params.Cuts = 0
    #model.Params.Heuristic = 0
    #model.Params.Presolve = 0
    # model.Params.PoolSolutionMode = 1
    # model.Params.SolutionNumber = 1000
    # model.Params.MIPFocus = 1

    #return ralaxed model
    if relax == True:
        # allvariables = model.getVars()
        # for var in model.getVars():
        #     print(var)
        #     var.vtype = GRB.CONTINUOUS
        for key in nodesVisitedByTruck.keys():
            nodesVisitedByTruck[key].vtype = GRB.CONTINUOUS
        for key in arcsVisitedByTruck.keys():
            arcsVisitedByTruck[key].vtype = GRB.CONTINUOUS
        for key in arcsVisitedByDrone.keys():
            arcsVisitedByDrone[key].vtype = GRB.CONTINUOUS
        model.write(
            'models\\' + "MTRPD_[Instance = {0}, Drones = {1}, Allowed Nr. of Drone Trips = {2}, Flight Range = {3}].lp".format(
                graph, K, Q, E))
        return model, constraints, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime
    else:
        model.write('models\\' + "MTRPD_[Instance = {0}, Drones = {1}, Allowed Nr. of Drone Trips = {2}, Flight Range = {3}].lp".format(graph, K, Q, E))
        return model, constraints, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime

def get_arc_list_of_csv(path):
    df = pd.read_csv(path)
    result = [
        (df['Node1'][ind], df['Node2'][ind], df['traveling time'][ind])
        for ind in df.index
    ]
    return result


def plot_graph(G, dictNodePos, *args):
    if len(args) != 0:
        nx.draw(G, with_labels=True, pos=dictNodePos, node_color = [color for color in args[0].values()])
    else:
        nx.draw(G, with_labels=True, pos=dictNodePos)
    arcLabels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos = dictNodePos, edge_labels = arcLabels)
    plt.show()

#return the list of arcs of the form (i, j, distance(i,j))
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

def solve_model(model,
                nodesVisitedByTruck,
                arcsVisitedByTruck,
                arcsVisitedByDrone,
                truckArrivingTime,
                droneArrivingTime,
                efficientSolvingGurobi,
                graph,
                droneSpeed,
                relax):
    #traveling time of truck and drone
    #get the traveling times (dict) --> (arc) : weight
    travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
    travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}

    #deactivate any automatic cuts, heuristics and presolve Gurobi techniques
    if efficientSolvingGurobi == False:
        model.setParam('Cuts', 0)
        model.setParam('ImpliedCuts', 0)
        model.setParam('CliqueCuts', 0)
        model.setParam('Heuristics', 0)
        model.setParam('Presolve', 0)
    #set Time limit: 20min
    model.setParam('TimeLimit', 60*60)
    #set log file
    model.setParam('LogFile', "Logs\\" + str(graph) + '.txt')

    #solve the model
    model.optimize()

    #solution dicts of the model
    chosenNodesVisitedByTruck = {} # node: 1
    chosenArcsVisitedByTruck = {} # (startNode, endNode, level): 1
    chosenArcsVisitedByDrone = {} # (startNode, endNode, drone, droneTrip, level): 1
    resultTruckArrivingTime = {} # level: value > 0
    resultDroneArrivingTime = {} # (endNode, droneTrip, level): value > 0
    # solution graph
    H = nx.DiGraph()
    # define the color of the nodes. red = truck node. green = drone node.
    colors = {}
    # the depot

    if relax != True:
        H.add_node(0)
        colors[0] = 'red'
        #add the truck nodes to the solution graph
        for (node, level) in nodesVisitedByTruck.keys():
            if nodesVisitedByTruck[(node, level)].X > 0.99:
                chosenNodesVisitedByTruck[(node, level)] = 1
                H.add_node(node)
                colors[node] = 'red'

        #add the drone nodes to the subdigraph
        for node in graph.nodes():
            if node not in H:
                H.add_node(node)
                colors[node] = 'green'

        #add the truck edges to the solution graph
        for (startNode, endNode, level) in arcsVisitedByTruck.keys():
            if arcsVisitedByTruck[(startNode, endNode, level)].X > 0.99:
                chosenArcsVisitedByTruck[(startNode, endNode, level)] = 1
                H.add_edge(startNode, endNode, weight = travelingTimeTruck[(startNode,endNode)])

        #add the drone edges to the solution graph
        for (startNode, endNode, drone, droneTrip, level) in arcsVisitedByDrone.keys():
            if arcsVisitedByDrone[(startNode, endNode, drone, droneTrip, level)].X > 0.99:
                chosenArcsVisitedByDrone[(startNode, endNode, drone, droneTrip, level)] = 1
                H.add_edge(startNode, endNode, weight = travelingTimeDrone[(startNode,endNode)])
                H.add_edge(endNode, startNode, weight = travelingTimeDrone[(startNode,endNode)])

        # get the truck arriving time in each level
        for l in truckArrivingTime.keys():
            if truckArrivingTime[l].X > 0:
                resultTruckArrivingTime[l] = truckArrivingTime[l].X
        # get the drone arriving time in each level in each droneTrip
        for (endNode, droneTrip, level) in droneArrivingTime.keys():
            if droneArrivingTime[(endNode, droneTrip, level)].X > 0:
                resultDroneArrivingTime[(endNode, droneTrip, level)] = droneArrivingTime[(endNode, droneTrip, level)].X

        # sort the results before returning them
        chosenNodesVisitedByTruck = dict(sorted(
            {item for item in chosenNodesVisitedByTruck.items()},
            key = lambda t : t[0][1],
            reverse = True
        ))
        chosenArcsVisitedByTruck = dict(sorted(
            {item
             for item in chosenArcsVisitedByTruck.items()},
            key=lambda t: t[0][2], reverse=True
        ))
        chosenArcsVisitedByDrone = dict(sorted(
            {item
             for item in chosenArcsVisitedByDrone.items()},
            key=lambda t: t[0][4], reverse=True
        ))
        resultTruckArrivingTime = dict(sorted(
            {item
             for item in resultTruckArrivingTime.items()},
            key=lambda t: t[0], reverse=True
        ))
        resultDroneArrivingTime = dict(sorted(
            {item
             for item in resultDroneArrivingTime.items()},
            key=lambda t: t[0][2], reverse=True
        ))
        result = {}
        result['nodesColors'] = colors
        result['truckNodes'] = chosenNodesVisitedByTruck
        result['truckArcs'] = chosenArcsVisitedByTruck
        result['droneArcs'] = chosenArcsVisitedByDrone
        result['truckArrivingTimes'] = resultTruckArrivingTime
        result['droneArrivingTimes'] = resultDroneArrivingTime
        result['objectiveValue'] = model.objVal
        result['optGap'] = model.MIPGap*100
        result['cpu'] = model.runtime
        return result


    else:
        for (node, level) in nodesVisitedByTruck.keys():
            if nodesVisitedByTruck[(node, level)].X > 0:
                chosenNodesVisitedByTruck[(node, level)] = nodesVisitedByTruck[(node, level)].X

        # add the truck edges to the solution graph
        for (startNode, endNode, level) in arcsVisitedByTruck.keys():
            if arcsVisitedByTruck[(startNode, endNode, level)].X > 0:
                chosenArcsVisitedByTruck[(startNode, endNode, level)] = arcsVisitedByTruck[
                    (startNode, endNode, level)].X

        # add the drone edges to the solution graph
        for (startNode, endNode, drone, droneTrip, level) in arcsVisitedByDrone.keys():
            if arcsVisitedByDrone[(startNode, endNode, drone, droneTrip, level)].X > 0:
                chosenArcsVisitedByDrone[(startNode, endNode, drone, droneTrip, level)] = arcsVisitedByDrone[
                    (startNode, endNode, drone, droneTrip, level)].X

        # get the truck arriving time in each level
        for l in truckArrivingTime.keys():
            if truckArrivingTime[l].X > 0:
                resultTruckArrivingTime[l] = truckArrivingTime[l].X
        # get the drone arriving time in each level in each droneTrip
        for (endNode, droneTrip, level) in droneArrivingTime.keys():
            if droneArrivingTime[(endNode, droneTrip, level)].X > 0:
                resultDroneArrivingTime[(endNode, droneTrip, level)] = droneArrivingTime[(endNode, droneTrip, level)].X

        #sort the results before returning them
        chosenNodesVisitedByTruck = dict(sorted(
            {item
            for item in chosenNodesVisitedByTruck.items()},
            key = lambda t: t[0][1], reverse=True
        ))
        chosenArcsVisitedByTruck = dict(sorted(
            {item
            for item in chosenArcsVisitedByTruck.items()},
            key = lambda t: t[0][2], reverse=True
        ))
        chosenArcsVisitedByDrone = dict(sorted(
            {item
            for item in chosenArcsVisitedByDrone.items()},
            key = lambda t: t[0][4], reverse=True
        ))
        resultTruckArrivingTime = dict(sorted(
            {item
            for item in resultTruckArrivingTime.items()},
            key = lambda t: t[0], reverse=True
        ))
        resultDroneArrivingTime = dict(sorted(
            {item
            for item in resultDroneArrivingTime.items()},
            key = lambda t: t[0][2], reverse=True
        ))

        result = {}
        result['nodesColors'] = colors
        result['truckNodes'] = chosenNodesVisitedByTruck
        result['truckArcs'] = chosenArcsVisitedByTruck
        result['droneArcs'] = chosenArcsVisitedByDrone
        result['truckArrivingTimes'] = resultTruckArrivingTime
        result['droneArrivingTimes'] = resultDroneArrivingTime
        result['objectiveValue'] = model.objVal
        result['optGap'] = model.MIPGap*100
        result['cpu'] = model.runtime
        return result



def main():
    #allnodes: Truck and drone nodes
    # nodeList_2customers = [i for i in range(0, 3, 1)]
    # nodeList_3customers = [i for i in range(0, 4, 1)]
    # nodeList_4customers = [i for i in range(0, 5, 1)]
    # nodeList_6customers = [i for i in range(0, 7, 1)]
    #
    # #positions of the nodes
    # #position list -> [(longitude, latitude)]
    # posList_2customers = [(1, 1), (3, 1), (3, 2)] #0, 1, 2
    # posList_3customers = [(1, 1), (3, 1), (3.5, 2), (3.5, 0)] #0, 1, 2, 3
    # posList_4customers = [(1, 1), (3, 1), (3, 1.5), (3, 0.5), (6, 1)] #0, 1, 2, 3, 4
    # posList_6customers = [(1, 1), (3, 1), (3, 1.5), (3, 0.5), (6, 1), (8, 1.25), (8, 0.75)] #0, 1, 2, 3, 4, 5, 6
    #
    # #position dict -> node : (longitude, latitude)
    # nodePos_2customers = {node:pos for (node, pos) in zip(nodeList_2customers, posList_2customers)}
    # nodePos_3customers = {node:pos for (node, pos) in zip(nodeList_3customers, posList_3customers)}
    # nodePos_4customers = {node:pos for (node, pos) in zip(nodeList_4customers, posList_4customers)}
    # nodePos_6customers = {node:pos for (node, pos) in zip(nodeList_6customers, posList_6customers)}

    #arcs (start, end, traveling time)

    # we add here r before the path to avoid having problems because of the slash, r is giving a raw stringraph. This means that for example
    # \U will not be interpreted as something special
    # arcList_2customers = get_arc_list_of_csv(r"Instances\MTRPD-2-customers.csv")
    # arcList_3customers = get_arc_list_of_csv(r"Instances\MTRPD-3-customers.csv")
    # arcList_4customers = get_arc_list_of_csv(r"Instances\MTRPD-4-customers.csv")
    # arcList_6customers = get_arc_list_of_csv(r"Instances\MTRPD-6-customers.csv")


    #
    # instance7NodesUniform = r"Instances\uniform-1-n7.txt"
    # instance7NodesUniformLocations, instance7NodesUniformArcs, graph7 = read_instance_file(instance7NodesUniform)
    # instance8NodesUniform = r'Instances\uniform-1-n8.txt'
    # instance8NodesUniformLocations, instance8NodesUniformArcs, graph8 = read_instance_file(instance8NodesUniform)
    # instance9NodesUniform = r'Instances\uniform-1-n9.txt'
    # instance9NodesUniformLocations, instance9NodesUniformArcs, graph9  = read_instance_file(instance9NodesUniform)
    # instance10NodesUniform = r'Instances\uniform-1-n10.txt'
    # instance10NodesUniformLocations, instance10NodesUniformArcs, graph10u  = read_instance_file(instance10NodesUniform)
    # instance10NodesCentroidUniform = r'Instances\centroid-uniform-1-n11.txt'
    # instance10NodesCentroidUniformLocations, instance10NodesCentroidUniformArcs, graph10cu  = read_instance_file(instance10NodesCentroidUniform)
    # instance11NodesUniform = r'Instances\uniform-1-n11.txt'
    # instance11NodesUniformLocations, instance11NodesUniformArcs, graph11u  = read_instance_file(instance11NodesUniform)
    #
    # #not solvable:
    # instance20NodesUniform = r'Instances\uniform-1-n20.txt'
    # instance20NodesUniformLocations, instance20NodesUniformArcs, graph20u  = read_instance_file(instance20NodesUniform)


    #Plot the graph
    #plot_graph(G10CentroidUniform, instance10NodesCentroidUniformLocations)

    #all variables:
    #instance = r"Instances\uniform-centroid-0-n8.txt"
    #instanceLocations, instanceArcs, graph = read_instance_file(instance)
    droneSpeed = 2 #{1, 2, 3}
    numberOfDrones = 2 #{1, 2, 3}
    numberOfTrips = 2 #per node per launch customer {1, 2, 3}
    #long distances
    #largeNumber = 100000
    largeNumber = 1000000
    #short distances
    #largeNumber = 100000
    #flightEndurance = np.ceil(2*max(nx.get_edge_attributes(graph, 'weight').values()))
    efficientSolvingGurobi = True
    relax = False


    #model, constraints, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime = \
    #    create_MTRPD(graph, numberOfDrones, numberOfTrips, largeNumber, flightEndurance, droneSpeed, relax)

    #
    # solutionGraph, \
    # nodeColors, \
    # modelObjValue,\
    # chosenNodesVisitedByTruck,\
    # chosenarcsVisitedByTruck,\
    # chosenarcsVisitedByDrone,\
    # resulttruckArrivingTime,\
    # resultDroneArrivingTime,\
    # sortedIndicesListOfCustomersTruckTrips,\
    # sortedIndicesListOfCustomersDroneTrips\
    #     =solve_model(model,
    #             nodesVisitedByTruck,
    #             arcsVisitedByTruck,
    #             arcsVisitedByDrone,
    #             truckArrivingTime,
    #             droneArrivingTime,
    #             efficientSolvingGurobi,
    #             graph,
    #             droneSpeed)
    #
    # plot_graph(solutionGraph, instanceLocations, nodeColors)
    # print("model run time", model.runtime)
    #
    # print("model gap", model.MIPGap*100)
    #instance:runtime
    results = {}
    relaxValuePerLevel = {}

    instancePath = r"Instances\illustrative example.txt"
    instance = instancePath[10:(len(instancePath) - 4)]
    print('-------> current Instance: ', instance)
    instanceLocations, instanceArcs, graph = read_instance_file(instancePath)
    travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
    travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}
    flightEndurance = np.ceil(2 * max(travelingTimeDrone.values()))
    # number of Levels / customers to be visited in the backward order
    numberOfLevels = graph.number_of_nodes() - 1
    model, constraints, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime = \
        create_MTRPD(graph, numberOfLevels, numberOfDrones, numberOfTrips, largeNumber, flightEndurance, droneSpeed,
                     relax)

    results[instance] = solve_model(model,
                                    nodesVisitedByTruck,
                                    arcsVisitedByTruck,
                                    arcsVisitedByDrone,
                                    truckArrivingTime,
                                    droneArrivingTime,
                                    efficientSolvingGurobi,
                                    graph,
                                    droneSpeed,
                                    relax)


    #uniform centroid - 10 nodes
    for i in range(10):
        instancePath = r"Instances\uniform-centroid-" + str(i) + "-n10.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        instanceLocations, instanceArcs, graph = read_instance_file(instancePath)
        travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
        travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}
        flightEndurance = np.ceil(2 * max(travelingTimeDrone.values()))
        # number of Levels / customers to be visited in the backward order
        numberOfLevels = graph.number_of_nodes() - 1
        model, constraints, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime = \
            create_MTRPD(graph, numberOfLevels, numberOfDrones, numberOfTrips, largeNumber, flightEndurance, droneSpeed, relax)

        results[instance] = solve_model(model,
                nodesVisitedByTruck,
                arcsVisitedByTruck,
                arcsVisitedByDrone,
                truckArrivingTime,
                droneArrivingTime,
                efficientSolvingGurobi,
                graph,
                droneSpeed,
                relax)
    #uniform centroid - 11 nodes
    for i in range(10):
        instancePath = r"Instances\uniform-centroid-1" + str(i) + "-n11.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        instanceLocations, instanceArcs, graph = read_instance_file(instancePath)
        travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
        travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}
        flightEndurance = np.ceil(2 * max(travelingTimeDrone.values()))
        # number of Levels / customers to be visited in the backward order
        numberOfLevels = graph.number_of_nodes() - 1
        model, constraints, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime = \
            create_MTRPD(graph, numberOfLevels, numberOfDrones, numberOfTrips, largeNumber, flightEndurance, droneSpeed,
                         relax)

        results[instance] = solve_model(model,
                                        nodesVisitedByTruck,
                                        arcsVisitedByTruck,
                                        arcsVisitedByDrone,
                                        truckArrivingTime,
                                        droneArrivingTime,
                                        efficientSolvingGurobi,
                                        graph,
                                        droneSpeed,
                                        relax)
    #uniform origin - 10 nodes
    for i in range(10):
        instancePath = r"Instances\uniform-origin-10" + str(i) + "-n10.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        instanceLocations, instanceArcs, graph = read_instance_file(instancePath)
        travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
        travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}
        flightEndurance = np.ceil(2 * max(travelingTimeDrone.values()))
        # number of Levels / customers to be visited in the backward order
        numberOfLevels = graph.number_of_nodes() - 1
        model, constraints, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime = \
            create_MTRPD(graph, numberOfLevels, numberOfDrones, numberOfTrips, largeNumber, flightEndurance, droneSpeed,
                         relax)

        results[instance] = solve_model(model,
                                        nodesVisitedByTruck,
                                        arcsVisitedByTruck,
                                        arcsVisitedByDrone,
                                        truckArrivingTime,
                                        droneArrivingTime,
                                        efficientSolvingGurobi,
                                        graph,
                                        droneSpeed,
                                        relax)
    #uniform origin - 11 nodes
    for i in range(10):
        instancePath = r"Instances\uniform-origin-11" + str(i) + "-n11.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        instanceLocations, instanceArcs, graph = read_instance_file(instancePath)
        travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
        travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}
        flightEndurance = np.ceil(2 * max(travelingTimeDrone.values()))
        # number of Levels / customers to be visited in the backward order
        numberOfLevels = graph.number_of_nodes() - 1
        model, constraints, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime = \
            create_MTRPD(graph, numberOfLevels, numberOfDrones, numberOfTrips, largeNumber, flightEndurance, droneSpeed,
                         relax)

        results[instance] = solve_model(model,
                                        nodesVisitedByTruck,
                                        arcsVisitedByTruck,
                                        arcsVisitedByDrone,
                                        truckArrivingTime,
                                        droneArrivingTime,
                                        efficientSolvingGurobi,
                                        graph,
                                        droneSpeed,
                                        relax)
    # clustered centroid - 10 nodes
    for i in range(10):
        instancePath = r"Instances\clustered-centroid-20" + str(i) + "-n10.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        instanceLocations, instanceArcs, graph = read_instance_file(instancePath)
        travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
        travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}
        flightEndurance = np.ceil(2 * max(travelingTimeDrone.values()))
        # number of Levels / customers to be visited in the backward order
        numberOfLevels = graph.number_of_nodes() - 1
        model, constraints, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime = \
            create_MTRPD(graph, numberOfLevels, numberOfDrones, numberOfTrips, largeNumber, flightEndurance, droneSpeed,
                         relax)

        results[instance] = solve_model(model,
                                        nodesVisitedByTruck,
                                        arcsVisitedByTruck,
                                        arcsVisitedByDrone,
                                        truckArrivingTime,
                                        droneArrivingTime,
                                        efficientSolvingGurobi,
                                        graph,
                                        droneSpeed,
                                        relax)
    # clustered centroid - 11 nodes
    for i in range(10):
        instancePath = r"Instances\clustered-centroid-21" + str(i) + "-n11.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        instanceLocations, instanceArcs, graph = read_instance_file(instancePath)
        travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
        travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}
        flightEndurance = np.ceil(2 * max(travelingTimeDrone.values()))
        # number of Levels / customers to be visited in the backward order
        numberOfLevels = graph.number_of_nodes() - 1
        model, constraints, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime = \
            create_MTRPD(graph, numberOfLevels, numberOfDrones, numberOfTrips, largeNumber, flightEndurance, droneSpeed,
                         relax)

        results[instance] = solve_model(model,
                                        nodesVisitedByTruck,
                                        arcsVisitedByTruck,
                                        arcsVisitedByDrone,
                                        truckArrivingTime,
                                        droneArrivingTime,
                                        efficientSolvingGurobi,
                                        graph,
                                        droneSpeed,
                                        relax)
    #clustered origin - 10 nodes
    for i in range(10):
        instancePath = r"Instances\clustered-origin-30" + str(i) + "-n10.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        instanceLocations, instanceArcs, graph = read_instance_file(instancePath)
        travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
        travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}
        flightEndurance = np.ceil(2 * max(travelingTimeDrone.values()))
        # number of Levels / customers to be visited in the backward order
        numberOfLevels = graph.number_of_nodes() - 1
        model, constraints, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime = \
            create_MTRPD(graph, numberOfLevels, numberOfDrones, numberOfTrips, largeNumber, flightEndurance, droneSpeed,
                         relax)

        results[instance] = solve_model(model,
                                        nodesVisitedByTruck,
                                        arcsVisitedByTruck,
                                        arcsVisitedByDrone,
                                        truckArrivingTime,
                                        droneArrivingTime,
                                        efficientSolvingGurobi,
                                        graph,
                                        droneSpeed,
                                        relax)
    #clustered origin - 11 nodes
    for i in range(10):
        # 11 nodes
        instancePath = r"Instances\clustered-origin-31" + str(i) + "-n11.txt"
        instance = instancePath[10:(len(instancePath) - 4)]
        print('-------> current Instance: ', instance)
        instanceLocations, instanceArcs, graph = read_instance_file(instancePath)
        travelingTimeTruck = nx.get_edge_attributes(graph, 'weight')
        travelingTimeDrone = {arc: int(time / droneSpeed) for (arc, time) in travelingTimeTruck.items()}
        flightEndurance = np.ceil(2 * max(travelingTimeDrone.values()))
        # number of Levels / customers to be visited in the backward order
        numberOfLevels = graph.number_of_nodes() - 1
        model, constraints, nodesVisitedByTruck, arcsVisitedByTruck, arcsVisitedByDrone, truckArrivingTime, droneArrivingTime = \
            create_MTRPD(graph, numberOfLevels, numberOfDrones, numberOfTrips, largeNumber, flightEndurance, droneSpeed,
                         relax)

        results[instance] = solve_model(model,
                                        nodesVisitedByTruck,
                                        arcsVisitedByTruck,
                                        arcsVisitedByDrone,
                                        truckArrivingTime,
                                        droneArrivingTime,
                                        efficientSolvingGurobi,
                                        graph,
                                        droneSpeed,
                                        relax)


    columns = ['Instance', 'CPU', 'GAP', 'Objective Value']
    resultsExcel = {col: [] for col in columns}
    for inst in results.keys():
        resultsExcel['Instance'].append(inst)
        resultsExcel['CPU'].append(results[inst]['cpu'])
        resultsExcel['GAP'].append(results[inst]['optGap'])
        resultsExcel['Objective Value'].append(results[inst]['objectiveValue'])

    with pd.ExcelWriter('results\\MTRPD_[DroneSpeed_=_{0},_NrDrones_=_{1},_DroneTrips_=_{2},_timeStamp_=_{3}].xlsx'.format(droneSpeed, numberOfDrones, numberOfTrips, time.time()), engine='openpyxl') as writer:
        df_resultsExcel = pd.DataFrame(resultsExcel)
        df_resultsExcel.to_excel(writer, sheet_name='results')

    fileName = 'MTRPD_[DroneSpeed_=_{0},_NrDrones_=_{1},_DroneTrips_=_{2},_timeStamp_=_{3}]'.format(
        droneSpeed, numberOfDrones, numberOfTrips, time.time())
    export_import_Results.exportAllResultsToTextFile(results, fileName)

    return results
    # plot solution graph
    #plot_graph(solutionGraph, nodesLocations, nodeColors)
    #print('done')





if __name__ == '__main__':
    main()