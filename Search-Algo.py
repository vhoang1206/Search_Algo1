from collections import defaultdict

class Graph:
    def __init__(self):
        # default dictionary to store graph
        self.graph = defaultdict(list)
        self.count_node = 0
    # function to add an edge to graph

    def addEdge(self, u, v):
        self.graph[u].append(v)

    # Function to print a BFS of graph
    def BFS(self, node):
        # Mark all the vertices as not visited
        visited = []
        # Create a queue for BFS
        queue = []
        # Mark the source node as visited and enqueue it
        queue.append(node)
        visited.append(node)
        while queue:
            # Dequeue a vertex from queue and print it
            s = queue.pop(0)
            print(s, end=" ")
            # Get All adjacent vertices of the dequeued vertex s. If a adjacent has not been visited, then mark it visited and enqueue it.
            for neighbour in self.graph[s]:
                if neighbour not in visited:
                    queue.append(neighbour)
                    visited.append(neighbour)

    # Function to print a DFS of graph
    def DFS(self, node):
        # Mark all the vertices as not visited
        visited = []
        # Create a stack for DFS
        stack = []
        # Mark the source node and push it
        stack.append(node)
        visited.append(node)

        while stack:
            # Pop a vertex from stack and print it
            s = stack.pop()
            print(s, end=" ")
            # mark it visited and push it.
            for neighbour in self.graph[s]:
                if neighbour not in visited:
                    visited.append(neighbour)
                    stack.append(neighbour)

    def DLS(self, source, target, maxDepth):
        if source == target:
            return True

        if maxDepth <= 0:
            return False

        for i in self.graph[source]:
            if(self.DLS(i, target, maxDepth-1)):
                return True
        return False

    def IDDFS(self, source, target):
        for i in range(10**5):
            if self.DLS(source, target, i):
                return True
        return False

def generateDirectedGraph(edges):
    graph = defaultdict(dict)
    for u, v, dist in edges:
        graph[u][v] = dist
    return graph

def dijkstra(graph, source):
    queue = [source]
    minDistances = {v: float("inf") for v in graph}
    minDistances[source] = 0
    predecessor = {}

    while queue:
        currentNode = queue.pop(0)
        for neighbor in graph[currentNode]:
            # get potential newDist from start to neighbor
            newDist = minDistances[currentNode] + graph[currentNode][neighbor]
            
            # if the newDist is shorter to reach neighbor updated to newDist
            if newDist < minDistances[neighbor]:
                minDistances[neighbor] = min(newDist, minDistances[neighbor])
                queue.append(neighbor)
                predecessor[neighbor] = currentNode

    return minDistances, predecessor

def UCS(graph, source, dest):
    minDistances, predecessor = dijkstra(graph, source)
    
    path = []
    currentNode = dest
    while currentNode != source:
        if currentNode not in predecessor:
            print("Path not reachable")
            break
        else:
            path.insert(0, currentNode)
            currentNode = predecessor[currentNode]
    path.insert(0, source)
    
    if dest in minDistances and minDistances[dest] != float("inf"):
        print('Shortest distance is ' + str(minDistances[dest]))
        print('And the path is ' + str(path))

def main():
    # Implementation

    g1 = Graph()
    g1.addEdge(0, 1)
    g1.addEdge(0, 2)
    g1.addEdge(1, 2)
    g1.addEdge(2, 0)
    g1.addEdge(2, 3)
    g1.addEdge(3, 3)

    g2 = Graph()
    g2.addEdge(0, 1)
    g2.addEdge(0, 2)
    g2.addEdge(1, 3)
    g2.addEdge(1, 4)
    g2.addEdge(2, 5)
    g2.addEdge(2, 6)

    weighted_edges = [['a', 'b', 6], ['a', 'c', 3], ['b', 'c', 1], ['c', 'b', 4], ['b', 'd', 2], ['c', 'd', 8], ['c', 'e', 2], ['d', 'e', 9], ['e', 'd', 7]]
    directed_weighted_graph = generateDirectedGraph(weighted_edges)

    # DFS and BFS
    print("BFS AND DFS: \n")
    print("Following is Breadth First Traversal (starting from vertex 2)")
    g1.BFS(2)
    print("\nFollowing is Depth First Traversal (starting from vertex 2)")
    g1.DFS(2)

    # DLS and IDDFS
    source = 0
    target = 6
    maxDepth = 3
    print("\n>>>DLS: ")
    if g2.DLS(source, target, maxDepth) == True:
        print("Solution is FOUND within max depth")
    else:
        print("Solution is NOT FOUND within max depth")
    print("\n>>>IDDFS: ")
    if g2.IDDFS(source, target) == True:
        print("Solution is FOUND", end = "\n")
    else:
        print("Solution is NOT FOUND", end = "\n") 

    # UCS 
    print(">>>UCS: ")
    UCS(directed_weighted_graph, 'a', 'd')

if __name__ == '__main__':
    main()