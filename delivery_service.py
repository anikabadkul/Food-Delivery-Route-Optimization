"""
UMass ECE 241 - Advanced Programming
Project 2 - Fall 2023
"""
import sys
from graph import Graph, Vertex
from priority_queue import PriorityQueue

class DeliveryService:
    def __init__(self) -> None:
        """
        Constructor of the Delivery Service class
        """
        self.city_map = Graph()
        self.MST = None

    def buildMap(self, filename: str) -> None:
        """

        :param filename:
        :return:
        """
        try:
            with open(filename, "r") as file:
                for i in file:
                    x1, x2, weight = map(int, i.strip().split('|')) # extracting edge information
                    self.city_map.addEdge(x1, x2, weight) #adding edges to the city map
                    self.city_map.addEdge(x2, x1, weight)

        except FileNotFoundError:
            print("File not found.")

    def isWithinServiceRange(self, restaurant: int, user: int, threshold: int) -> bool:
        """

        :param restaurant:
        :param user:
        :param threshold:
        :return:
        """
        if user not in self.city_map.vertList: # checks whether the node exists
            return False
        stack = [(self.city_map.vertList[restaurant], threshold)] # inital stack with the restaurant and threshold
        visited = set() # keeps track of visied nodes
        while stack:
            recent, remaining_threshold = stack.pop()
            visited.add(recent.getId()) # marks as visited
            if recent.getId() == user: # whether the current node matches the user
                return True
            # visited.add(recent.getId()) # marks as visited
            for next in recent.getConnections(): # visits the neighbors basd on threshold and visited status
                corner_weight = recent.getWeight(next)
                if next.getId() not in visited and corner_weight <= remaining_threshold:
                    stack.append((next, remaining_threshold - corner_weight))
            # if not stack and recent.getId != user: # if the stack is emty and the current node doeesnt match the user,go explore other paths
            #     for next in recent.getConnections():
            #         corner_weight = recent.getWeight(next)
            #         if next.getId() not in visited and corner_weight <= threshold:
            #             stack.append((next, threshold - corner_weight))
        return False

    def buildMST(self, restaurant: int) -> bool:
        """

        :param restaurant:
        :return:
        """
        if restaurant not in self.city_map.vertList: # checks whether the restaurant node exists
            return False
        begin = self.city_map.vertList[restaurant] # this begins MST construction from the provided restaurant node
        pq = PriorityQueue()
        begin.setDistance(0) # sets initial distance to 0 for starting vertex
        pq.buildHeap([(x.getDistance(),x)for x in self.city_map]) # creating a priority queue
        visited = set() # sto keep track of visited vertices
        for _ in range(len(pq.heapArray)-1): # loops through vertices for MST constructions
            current_vertex = pq.delMin() #gets the minimum distance vertex
            visited.add(current_vertex.getId())  #adds it to the visited set
            for next_vertex in current_vertex.getConnections(): # goes to neghbors and updates distances
                cost = current_vertex.getWeight(next_vertex)
                if next_vertex.getId() not in visited and cost < next_vertex.getDistance(): #checks whether the neighbors are unvisited and the cost is less than current distance
                    next_vertex.setDistance(cost) #updates distance, predecessor and priority queue
                    next_vertex.setPred(current_vertex)
                    pq.decreaseKey(next_vertex, cost)

        self.MST = Graph() # initializing the MST Graph
        for v in self.city_map: # adds the edges to te MST based on the calculatd presdecessors
            previous = v.getPred()
            if previous is not None:
                cost = v.getWeight(previous)
                self.MST.addEdge(previous.getId(), v.getId(), cost)
        return True
    def minimalDeliveryTime(self, restaurant: int, user: int) -> int:
        """

        :param restaurant:
        :param user:
        :return:
        """
        if user not in self.MST.vertList or restaurant not in self.MST.vertList: #checks whether the user or restaurant nodes does not exist
            return -1
        initial_vertex = self.MST.vertList[restaurant] #sets the initial distance to 0
        initial_vertex.setDistance(0)
        pq = PriorityQueue() # prepare the priority queue based on the distance of vertices in MST
        pq.buildHeap([(v.getDistance(), v) for v in self.MST])
        while pq.isEmpty() == False: #traverse the vertices to find the minal dstance using dijkstra'algorithm
            current_vertex = pq.delMin()
            for next in current_vertex.getConnections():
                new_path = current_vertex.getDistance() + current_vertex.getWeight(next)
                if new_path < next.getDistance():
                    next.setDistance(new_path) # updates the distance and predecessor if a shorter path exists
                    next.setPred(current_vertex)
                    pq.decreaseKey(next, new_path)
        return self.MST.vertList[user].getDistance() # returns the mininal delievery time
    
    def findDeliveryPath(self, restaurant: int, user: int) -> str:
        """

        :param restaurant:
        :param user:
        :return:
        """
        if restaurant not in self.city_map.vertList or user not in self.city_map.vertList: #checks whether resaurant or user nodes doesnt xist in the map
            return "INVALID"
        initial = self.city_map.vertList[restaurant]
        goal = self.city_map.vertList[user]
        visited = {initial: None}
        queue = [(initial, 0)]
        paths = {initial: []}
        while len(queue) > 0: # perform BFS
            current, time = queue.pop(0)
            if current == goal:
                path = paths[current] + [current.getId()] # generate path string once the target is reached
                path_str = '->'.join(map(str, path))
                return f"{path_str}({time})"
            for neighbor in current.getConnections(): # explore the neighbors of the current node
                update = time + current.getWeight(neighbor) # updates time for each neighbor
                if neighbor not in visited or (visited[neighbor] is not None and update < visited[neighbor]): # checks if the neighbor hasnt been visitd or a shorter path exists
                    visited[neighbor] = update
                    paths[neighbor] = paths[current] + [current.getId()]
                    queue.append((neighbor, update))
            queue.sort(key=lambda x: x[1] if visited[x[0]] is not None else float('inf')) # sorts the queue based on time
        return "INVALID"

    def findDeliveryPathWithDelay(self, restaurant: int, user: int, delay_info: dict[int, int]) -> str:
        """

        :param restaurant:
        :param user:
        :param delay_info:
        :return:
        """
        if restaurant not in self.city_map.vertList or user not in self.city_map.vertList: #checks whether resaurant or user nodes doesnt xist in the map
            return "INVALID"
        initial = self.city_map.vertList[restaurant]
        goal = self.city_map.vertList[user]
        visited = {initial: None}
        queue = [(initial, 0)]
        paths = {initial: []}
        while queue:  # perform BFS
            current, time = queue.pop(0)
            if current == goal:
                path = paths[current] + [current.getId()]  # generate path string once the target is reached
                path_str = '->'.join(map(str, path))
                return f"{path_str}({time})"
            for neighbor in current.getConnections():  # explore the neighbors of the current node
                update = time + current.getWeight(neighbor) + delay_info.get(neighbor.getId(), 0)  # updates time for each neighbor
                if neighbor not in visited or (visited[neighbor] is not None and update < visited[neighbor]):  # checks if the neighbor hasnt been visitd or a shorter path exists
                    visited[neighbor] = update
                    paths[neighbor] = paths[current] + [current.getId()]
                    queue.append((neighbor, update))
            queue.sort(key=lambda x: x[1] if visited[x[0]] is not None else float('inf'))  # sorts the queue based on time
        return "INVALID"

    ## DO NOT MODIFY CODE BELOW!
    @staticmethod
    def nodeEdgeWeight(v):
        return sum([w for w in v.connectedTo.values()])

    @staticmethod
    def totalEdgeWeight(g):
        return sum([DeliveryService.nodeEdgeWeight(v) for v in g]) // 2

    @staticmethod
    def checkMST(g):
        for v in g:
            v.color = 'white'

        for v in g:
            if v.color == 'white' and not DeliveryService.DFS(g, v):
                return 'Your MST contains circles'
        return 'MST'

    @staticmethod
    def DFS(g, v):
        v.color = 'gray'
        for nextVertex in v.getConnections():
            if nextVertex.color == 'white':
                if not DeliveryService.DFS(g, nextVertex):
                    return False
            elif nextVertex.color == 'black':
                return False
        v.color = 'black'

        return True

# NO MORE TESTING CODE BELOW!
# TO TEST YOUR CODE, MODIFY test_delivery_service.py
