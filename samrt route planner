import java.util.*;

public class SmartRoutePlanner {

    // Edge node for custom adjacency list
    static class EdgeNode {
        int to;
        int weight;
        EdgeNode next;

        EdgeNode(int to, int weight, EdgeNode next) {
            this.to = to;
            this.weight = weight;
            this.next = next;
        }
    }

    // Graph implemented using custom adjacency list
    static class Graph {
        private String[] names;
        private EdgeNode[] heads;
        private int vertexCount;

        Graph(int capacity) {
            names = new String[capacity];
            heads = new EdgeNode[capacity];
            vertexCount = 0;
        }

        // Add a new location
        int addLocation(String name) {

            if (vertexCount == names.length) {
                return -1;
            }

            if (getIndex(name) != -1) {
                return -2;
            }

            names[vertexCount] = name;
            return vertexCount++;
        }

        // Find location index
        int getIndex(String name) {

            for (int i = 0; i < vertexCount; i++) {

                if (names[i].equalsIgnoreCase(name)) {
                    return i;
                }
            }

            return -1;
        }

        // Add an undirected road
        boolean addRoad(String a, String b, int distance) {

            int u = getIndex(a);
            int v = getIndex(b);

            if (u == -1 || v == -1 || u == v || distance <= 0) {
                return false;
            }

            if (hasEdge(u, v)) {
                return false;
            }

            heads[u] = new EdgeNode(v, distance, heads[u]);
            heads[v] = new EdgeNode(u, distance, heads[v]);

            return true;
        }

        // Check duplicate road
        private boolean hasEdge(int u, int v) {

            EdgeNode current = heads[u];

            while (current != null) {

                if (current.to == v) {
                    return true;
                }

                current = current.next;
            }

            return false;
        }

        // Display graph
        void displayGraph() {

            if (vertexCount == 0) {
                System.out.println("No locations added.");
                return;
            }

            System.out.println("\n--- Road Network ---");

            for (int i = 0; i < vertexCount; i++) {

                System.out.print(names[i] + " -> ");

                EdgeNode current = heads[i];

                if (current == null) {
                    System.out.print("No roads");
                }

                while (current != null) {

                    System.out.print(
                        names[current.to]
                        + "("
                        + current.weight
                        + " km)"
                    );

                    if (current.next != null) {
                        System.out.print(" -> ");
                    }

                    current = current.next;
                }

                System.out.println();
            }
        }

        // ---------------- BFS ----------------

        List<Integer> bfs(int source, int destination) {

            boolean[] visited = new boolean[vertexCount];

            int[] parent = new int[vertexCount];

            Arrays.fill(parent, -1);

            Queue<Integer> queue = new ArrayDeque<>();

            queue.add(source);
            visited[source] = true;

            while (!queue.isEmpty()) {

                int u = queue.remove();

                if (u == destination) {
                    break;
                }

                EdgeNode current = heads[u];

                while (current != null) {

                    if (!visited[current.to]) {

                        visited[current.to] = true;

                        parent[current.to] = u;

                        queue.add(current.to);
                    }

                    current = current.next;
                }
            }

            return buildPath(
                parent,
                source,
                destination,
                visited[destination]
            );
        }

        // ---------------- DFS ----------------

        boolean dfsSearch(
            int u,
            int destination,
            boolean[] visited,
            List<Integer> path
        ) {

            visited[u] = true;

            path.add(u);

            if (u == destination) {
                return true;
            }

            EdgeNode current = heads[u];

            while (current != null) {

                if (!visited[current.to]) {

                    if (dfsSearch(
                        current.to,
                        destination,
                        visited,
                        path
                    )) {
                        return true;
                    }
                }

                current = current.next;
            }

            path.remove(path.size() - 1);

            return false;
        }

        List<Integer> dfs(int source, int destination) {

            boolean[] visited = new boolean[vertexCount];

            List<Integer> path = new ArrayList<>();

            if (dfsSearch(
                source,
                destination,
                visited,
                path
            )) {
                return path;
            }

            return Collections.emptyList();
        }

        // ---------------- Dijkstra ----------------

        static class State implements Comparable<State> {

            int vertex;
            int distance;

            State(int vertex, int distance) {

                this.vertex = vertex;
                this.distance = distance;
            }

            @Override
            public int compareTo(State other) {

                return Integer.compare(
                    this.distance,
                    other.distance
                );
            }
        }

        List<Integer> dijkstra(
            int source,
            int destination,
            int[] resultDistance
        ) {

            final int INF = Integer.MAX_VALUE / 4;

            int[] distance = new int[vertexCount];

            int[] parent = new int[vertexCount];

            boolean[] processed = new boolean[vertexCount];

            Arrays.fill(distance, INF);

            Arrays.fill(parent, -1);

            PriorityQueue<State> pq =
                new PriorityQueue<>();

            distance[source] = 0;

            pq.add(
                new State(source, 0)
            );

            while (!pq.isEmpty()) {

                State current = pq.poll();

                int u = current.vertex;

                if (processed[u]) {
                    continue;
                }

                processed[u] = true;

                if (u == destination) {
                    break;
                }

                EdgeNode edge = heads[u];

                while (edge != null) {

                    int v = edge.to;

                    if (
                        !processed[v]
                        &&
                        distance[u] + edge.weight
                        < distance[v]
                    ) {

                        distance[v] =
                            distance[u]
                            + edge.weight;

                        parent[v] = u;

                        pq.add(
                            new State(
                                v,
                                distance[v]
                            )
                        );
                    }

                    edge = edge.next;
                }
            }

            if (distance[destination] == INF) {

                resultDistance[0] = -1;

                return Collections.emptyList();
            }

            resultDistance[0] =
                distance[destination];

            return buildPath(
                parent,
                source,
                destination,
                true
            );
        }

        // Build path using parent array
        private List<Integer> buildPath(
            int[] parent,
            int source,
            int destination,
            boolean reachable
        ) {

            if (!reachable) {
                return Collections.emptyList();
            }

            LinkedList<Integer> path =
                new LinkedList<>();

            int current = destination;

            while (current != -1) {

                path.addFirst(current);

                if (current == source) {
                    break;
                }

                current = parent[current];
            }

            if (path.getFirst() != source) {

                return Collections.emptyList();
            }

            return path;
        }

        String getName(int index) {
            return names[index];
        }

        int size() {
            return vertexCount;
        }
    }

    // Print route
    static void printPath(
        Graph graph,
        List<Integer> path
    ) {

        for (int i = 0; i < path.size(); i++) {

            System.out.print(
                graph.getName(path.get(i))
            );

            if (i < path.size() - 1) {
                System.out.print(" -> ");
            }
        }

        System.out.println();
    }

    // Display locations
    static void showLocations(Graph graph) {

        System.out.println(
            "\nAvailable Locations:"
        );

        for (int i = 0; i < graph.size(); i++) {

            System.out.println(
                (i + 1)
                + ". "
                + graph.getName(i)
            );
        }
    }

    // Sample data
    static void addSampleData(Graph graph) {

        String[] locations = {
            "Mumbai",
            "Pune",
            "Nashik",
            "Satara",
            "Aurangabad",
            "Kolhapur",
            "Nagpur"
        };

        for (String location : locations) {
            graph.addLocation(location);
        }

        graph.addRoad("Mumbai", "Pune", 50);
        graph.addRoad("Mumbai", "Nashik", 30);
        graph.addRoad("Pune", "Nashik", 40);
        graph.addRoad("Pune", "Satara", 70);
        graph.addRoad("Nashik", "Aurangabad", 60);
        graph.addRoad("Satara", "Kolhapur", 80);
        graph.addRoad("Nashik", "Nagpur", 100);
    }

    // ---------------- MAIN ----------------

    public static void main(String[] args) {

        Scanner sc = new Scanner(System.in);

        Graph graph = new Graph(50);

        // Load sample road network
        addSampleData(graph);

        while (true) {

            System.out.println(
                "\n========== SMART ROUTE PLANNER =========="
            );

            System.out.println("1. Add Location");
            System.out.println("2. Add Road");
            System.out.println("3. Display Road Network");
            System.out.println("4. BFS Route Search");
            System.out.println("5. DFS Route Search");
            System.out.println(
                "6. Shortest Route (Dijkstra)"
            );
            System.out.println("7. Show Locations");
            System.out.println("8. Exit");

            System.out.print("Enter choice: ");

            int choice;

            try {

                choice =
                    Integer.parseInt(
                        sc.nextLine().trim()
                    );

            } catch (Exception e) {

                System.out.println(
                    "Please enter a valid number."
                );

                continue;
            }

            switch (choice) {

                // Add location
                case 1:

                    System.out.print(
                        "Enter location name: "
                    );

                    String name =
                        sc.nextLine().trim();

                    int result =
                        graph.addLocation(name);

                    if (result >= 0) {

                        System.out.println(
                            "Location added successfully."
                        );

                    } else if (result == -2) {

                        System.out.println(
                            "Location already exists."
                        );

                    } else {

                        System.out.println(
                            "Graph capacity is full."
                        );
                    }

                    break;

                // Add road
                case 2:

                    System.out.print(
                        "Enter first location: "
                    );

                    String a =
                        sc.nextLine().trim();

                    System.out.print(
                        "Enter second location: "
                    );

                    String b =
                        sc.nextLine().trim();

                    System.out.print(
                        "Enter distance in km: "
                    );

                    int d;

                    try {

                        d =
                            Integer.parseInt(
                                sc.nextLine().trim()
                            );

                    } catch (Exception e) {

                        System.out.println(
                            "Invalid distance."
                        );

                        break;
                    }

                    if (
                        graph.addRoad(a, b, d)
                    ) {

                        System.out.println(
                            "Road added successfully."
                        );

                    } else {

                        System.out.println(
                            "Road not added."
                        );
                    }

                    break;

                // Display graph
                case 3:

                    graph.displayGraph();

                    break;

                // BFS / DFS / Dijkstra
                case 4:
                case 5:
                case 6:

                    System.out.print(
                        "Enter source: "
                    );

                    String sourceName =
                        sc.nextLine().trim();

                    System.out.print(
                        "Enter destination: "
                    );

                    String destinationName =
                        sc.nextLine().trim();

                    int source =
                        graph.getIndex(sourceName);

                    int destination =
                        graph.getIndex(destinationName);

                    if (
                        source == -1
                        ||
                        destination == -1
                    ) {

                        System.out.println(
                            "Invalid source or destination."
                        );

                        break;
                    }

                    // BFS
                    if (choice == 4) {

                        List<Integer> path =
                            graph.bfs(
                                source,
                                destination
                            );

                        if (path.isEmpty()) {

                            System.out.println(
                                "No route found using BFS."
                            );

                        } else {

                            System.out.print(
                                "BFS route: "
                            );

                            printPath(
                                graph,
                                path
                            );
                        }
                    }

                    // DFS
                    else if (choice == 5) {

                        List<Integer> path =
                            graph.dfs(
                                source,
                                destination
                            );

                        if (path.isEmpty()) {

                            System.out.println(
                                "No route found using DFS."
                            );

                        } else {

                            System.out.print(
                                "DFS route: "
                            );

                            printPath(
                                graph,
                                path
                            );
                        }
                    }

                    // Dijkstra
                    else {

                        int[] distance =
                            new int[1];

                        List<Integer> path =
                            graph.dijkstra(
                                source,
                                destination,
                                distance
                            );

                        if (path.isEmpty()) {

                            System.out.println(
                                "No route exists."
                            );

                        } else {

                            System.out.print(
                                "Shortest route: "
                            );

                            printPath(
                                graph,
                                path
                            );

                            System.out.println(
                                "Total distance: "
                                + distance[0]
                                + " km"
                            );

                            System.out.println(
                                "Algorithm: "
                                + "Dijkstra + Priority Queue"
                            );
                        }
                    }

                    break;

                // Locations
                case 7:

                    showLocations(graph);

                    break;

                // Exit
                case 8:

                    System.out.println(
                        "Thank you for using "
                        + "Smart Route Planner."
                    );

                    sc.close();

                    return;

                default:

                    System.out.println(
                        "Invalid choice."
                    );
            }
        }
    }
}
