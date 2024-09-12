import math
import random

class Node:
    def __init__(self, xOrigin=0.0, yOrigin=0.0, parental=-1, posIn=-100, inGoalIn=0, costIn=0.0):
        self.cord = [xOrigin, yOrigin]
        self.parent = parental
        self.pos = posIn
        self.inGoal = inGoalIn
        self.cost = costIn

class RRTAR:
    def __init__(self):
        self.goals = []
        self.rad = 0.0
        self.closePenalty = 0.0
        self.searchTree = []

    def check_in_goal(self, x, y):
        for i, elem in enumerate(self.goals):
            dist = (elem[0] - x) ** 2 + (elem[1] - y) ** 2
            if dist < self.rad:
                return i + 1
        return 0

    def closest(self, search_tree, x, y):
        min_dist = float('inf')
        closest_node = None
        for elem in search_tree:
            dist = (elem.cord[0] - x) ** 2 + (elem.cord[1] - y) ** 2
            if dist < min_dist and not elem.inGoal:
                min_dist = dist
                closest_node = elem
        return closest_node


    def steer(self, x, y, node, count):
        # Calculate the distance between the current node's coordinates and the given coordinates
        distance = math.sqrt((x - node.cord[0]) ** 2 + (y - node.cord[1]) ** 2)

        if distance <= self.rad:
            # If the distance is within the radius, create a new node at the given coordinates without projection
            new_node = Node(x, y, node.pos, count, self.check_in_goal(x, y), distance)
            return new_node
        else:
            # Calculate the angle between the current node's coordinates and the projected coordinates
            angle = math.atan2(y - node.cord[1], x - node.cord[0])

            # Calculate the projected x and y coordinates on the circle
            projectedX = node.cord[0] + self.rad * math.cos(angle)
            projectedY = node.cord[1] + self.rad * math.sin(angle)

            # Create a new node with the projected coordinates and set its parent to the input node
            projected_node = Node(projectedX, projectedY, node.pos, count, self.check_in_goal(projectedX, projectedY), self.rad)
            return projected_node

    def calc_cost(self, input_node):
        ans = input_node.cost
        while input_node.parent > -1:
            input_node = self.searchTree[input_node.parent]
            ans += input_node.cost
        return ans


    def rewire(self, count, current=float('inf')):
        if current == float('inf'):
            current = count

        withRad = []
        costs = []
        dists = []
        parents = []
        indexOfNodes = []

        x = self.searchTree[current].cord[0]
        y = self.searchTree[current].cord[1]

        # Initial search - normal RRT*
        for i in range(count):
            if i == current:
                continue
            dist = math.sqrt((self.searchTree[i].cord[0] - x) ** 2 + (self.searchTree[i].cord[1] - y) ** 2)
            if dist <= 1.5 * self.rad:
                withRad.append(i)
                cost = self.calc_cost(self.searchTree[i])
                costs.append(cost)
                dists.append(dist)
                parents.append(self.searchTree[i].parent)
                indexOfNodes.append(i)

        # Adding parent penalty for close neighbor (alg 1 or RRT*-AR)
        costRewire = list(costs)
        mySet = set()
        for i in range(len(costs)):
            parent = parents[i]
            foundIndex = indexOfNodes.index(parent) if parent in indexOfNodes else -1
            if foundIndex != -1 and i not in mySet:
                costRewire[foundIndex] += self.closePenalty
                mySet.add(i)

        minNeighbor = -1
        minCost = float('inf')
        for i in range(len(costs)):
            if costRewire[i] + dists[i] < minCost:
                minCost = costRewire[i] + dists[i]
                minNeighbor = withRad[i]

        self.searchTree[current].parent = minNeighbor

        permutation = sorted(range(len(costs)), key=lambda x: costs[x])
        limiter = True
        for i in permutation:
            if minCost + dists[i] < costs[i] and limiter:
                self.searchTree[withRad[i]].parent = current
                minCost += self.closePenalty
                limiter = False
            elif minCost + dists[i] < costs[i]:
                self.searchTree[withRad[i]].parent = current

    def runRRT(self, initialX, initialY, maxNodesIn, radIn, closePenaltyIn, og, goalsIn):
        xlen = len(og)
        ylen = len(og[0])
        self.goals = goalsIn
        self.rad = radIn
        self.closePenalty = closePenaltyIn
        origin = Node(initialX, initialY)
        self.searchTree = [origin]
        count = 1

        while count < maxNodesIn:
            x = random.uniform(0, xlen)
            y = random.uniform(0, ylen)
            
            if og[int(x)][int(y)]:
                continue
            
            close = self.closest(self.searchTree, x, y)
            Xnew = self.steer(x, y, close, count)
            
            if og[int(Xnew.cord[0])][int(Xnew.cord[1])]:
                continue
            
            self.searchTree.append(Xnew)
            self.rewire(count)

            count += 1

        # Adds every branch in the vector
        full_tree = []
        for node in self.searchTree:
            temp = node.cord[:]
            if node.parent >= 0:
                temp.append(float(node.parent))
            else:
                temp.append(-1.0)
            full_tree.append(temp)

        sol = [full_tree]

        # Adds branches that connect to the goal
        for node in self.searchTree:
            if node.inGoal > 0:
                parent_index = node.parent
                sol_branch = [self.goals[node.inGoal - 1], node.cord[:]]
                while parent_index >= 0:
                    node_inner = self.searchTree[parent_index]
                    sol_branch.append(node_inner.cord[:])
                    parent_index = node_inner.parent
                sol_branch.reverse()
                sol.append(sol_branch)

        return sol