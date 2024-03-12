import java.util.*;
import java.io.*;

public class Graph {

    public Map<String, Town> graph;
    public LinkedList<String> path;

    Graph() {
        graph = new HashMap<String, Town>();
        path = new LinkedList<String>();
    }

    void addTown(Town t) {
        graph.put(t.townName, t);
    }

    void addAdjacency(String t1, String t2) {
        graph.get(t1).addEdge(t2);
        graph.get(t2).addEdge(t1);
    }

    //returns Euclidean distance between two locations
    //used to weight each edge and as the heuristic in A* search
    double distance(String t1, String t2) {
        return Math.sqrt(Math.pow(graph.get(t2).lat - graph.get(t1).lat, 2) +
                Math.pow(graph.get(t2).lon - graph.get(t1).lon, 2));
    }

    //sorts a list by f value
    LinkedList<String> sortQ(LinkedList<String> list) {
        for (int i = 1; i < list.size(); ++i) {
            String key = list.get(i);
            int j = i - 1;
            while (j >= 0 && graph.get(list.get(j)).f > graph.get(list.get(i)).f) {
                list.set(j + 1, list.get(j));
                j = j - 1;
            }
            list.set(j + 1, key);
        }
        return list;
    }

    boolean breadthFirst(String start, String destination) {
        //store parent/child relationship for path tracking
        Map<String, String> d = new HashMap<String, String>();
        //create list of visited nodes and queue of adjacent nodes
        LinkedList<String> visited = new LinkedList<String>();
        Queue<String> queue = new LinkedList<String>();
        visited.add(start);
        queue.add(start);
        while (queue.peek() != null) {
            String town = queue.poll();
            //get adjacent towns, if not visited then enqueue and add to visited
            for (String tAdj : graph.get(town).adjTowns) {
                if (!visited.contains(tAdj)) {
                    d.put(tAdj, town);
                    visited.add(tAdj);
                    queue.add(tAdj);
                    if (town.equals(destination)) {
                        path.add(destination);
                        while (!town.equals(start)) {
                            town = d.get(town);
                            path.add(town);
                        }
                        return true;
                    }
                }
            }
        }
        return false;
    }

    boolean depthFirst(String start, String destination) {
        //store parent/child relationship for path tracking
        Map<String, String> d = new HashMap<String, String>();
        //create empty list of visited nodes and stack of nodes to be visited
        LinkedList<String> visited = new LinkedList<String>();
        Stack<String> stack = new Stack<String>();

        stack.add(start);
        while (!stack.empty()) {
            //remove current location from stack
            String town = stack.pop();
            if (!visited.contains(town)) {
                visited.add(town);
                for (String tAdj : graph.get(town).adjTowns) {
                    stack.add(tAdj);
                    //track adjacency
                    graph.get(tAdj).parent = town;
                    if (tAdj.equals(destination)) {
                        town = destination;
                        path.addFirst(town);
                        while (!town.equals(start)) {
                            town = graph.get(town).parent;
                            path.addFirst(town);
                        }
                        return true;
                    }
                }
            }
        }
        return false;
    }

    //recursive IDDFS function
    //source: https://www.geeksforgeeks.org/iterative-deepening-searchids-iterative-deepening-depth-first-searchiddfs/
    boolean IDDFS(String root, String target, int maxDepth) {
        for (int i = 0; i <= maxDepth; i++) {
            if (DLS(root, target, i)) {
                path.add(root);
                return true;
            }
        }
        return false;
    }

    //DLS (Depth-Limited Search) method used by IDDFS
    boolean DLS(String root, String target, int limit) {
        //base cases, target or max depth reached
        if (root.equals(target)) {
            return true;
        }
        //stop if max depth reached
        if (limit <= 0) {
            return false;
        }
        for (String adj : graph.get(root).adjTowns) {
            if (DLS(adj, target, limit - 1)) {
                path.add(adj);
                return true;
            }
        }
        return false;
    }

    /*
    returns path found from A* traversal
    f : value equal to the sum of g and h,
    g : total movement cost of path from starting location to a given town
    h : estimated movement cost from a given town to destination
    d : Map containing parent/child relationships for final path finding
    */
    boolean aStar(String root, String target) {
        LinkedList<String> open = new LinkedList<String>();
        LinkedList<String> closed = new LinkedList<String>();
        Map<String, String> d = new HashMap<String, String>();
        String node;
        open.add(root);

        while (!open.isEmpty()) {
            //remove node with lowest f value on open list
            node = open.pollFirst();
            for (String t : graph.get(node).adjTowns) {
                //skip if town exists within open or closed lists AND has a smaller f value
                if (open.contains(t) && graph.get(t).f < (distance(t, node) + graph.get(node).g) + distance(t, target)) {
                    continue;
                }
                else if (closed.contains(t) && graph.get(t).f < (distance(t, node) + graph.get(node).g) + distance(t, target)) {
                    continue;
                }
                graph.get(t).g = distance(t, node) + graph.get(node).g;
                graph.get(t).h = distance(t, target);
                graph.get(t).f = graph.get(t).g + graph.get(t).h;
                //initialize parent child relationship and add to open list
                d.put(t, node);
                open.add(t);
                //stop search if destination found
                //populates and returns list containing final path
                if (t.equals(target)) {
                    path.add(t);
                    while (!t.equals(root)) {
                        t = d.get(t);
                        path.add(t);
                    }
                    return true;
                }
                //sort queue from smallest to largest f value
                open = sortQ(open);
                closed.add(node);
            }
        }
        //return false if no valid path found;
        return false;
    }

    /*
    returns path found from best first traversal
    uses same heuristic as A*
    */
    boolean bestFirst(String root, String target) {
        LinkedList<String> visited = new LinkedList<String>();
        Stack<String> finalPath = new Stack<String>();
        Map<String, String> d = new HashMap<String, String>();

        //create PriorityQueue of towns sorted by smallest f value to largest
        PriorityQueue<String> pq = new PriorityQueue<String>(new Comparator<String>() {
            public int compare(String t1, String t2) {
                if (graph.get(t1).f < graph.get(t2).f) {
                    return -1;
                }
                else return 1;
            }
        });

        pq.add(root);
        while (!pq.isEmpty()) {
            String t = pq.poll();
            //generate and return final path if target found
            if (t.equals(target)) {
                finalPath.add(t);
                while (!t.equals(root)) {
                    t = d.get(t);
                    finalPath.add(t);
                }
                for (String e : finalPath) {
                    path.add(e);
                }
                return true;
            }
            else {
                for (String tAdj : graph.get(t).adjTowns) {
                    if (!visited.contains(tAdj)) {
                        //initialize heuristic, parent child relationship, add to visited list/pq
                        graph.get(tAdj).g = distance(t, tAdj) + graph.get(t).g;
                        graph.get(tAdj).h = distance(t, target);
                        graph.get(tAdj).f = graph.get(t).g + graph.get(t).h;
                        d.put(tAdj, t);
                        visited.add(tAdj);
                        pq.add(tAdj);
                    }
                }
            }
        }
        return false;
    }
}

//main program logic
class Main {

    //populates graph from files provided
    public static void buildGraph(Graph graph) {
        try {
            Scanner s1 = new Scanner(new File("coordinates.csv"));
            while (s1.hasNextLine()) {
                String[] splitStr = s1.nextLine().split(",");
                graph.addTown(new Town(splitStr[0], Double.parseDouble(splitStr[1]), Double.parseDouble(splitStr[2])));
            }
            s1.close();
        } catch (FileNotFoundException e1) {
            System.out.print("e1: File not found");
            return;
        }
        try {
            Scanner s2 = new Scanner(new File("adjacencies.txt"));
            while (s2.hasNextLine()) {
                String[] splitStr = s2.nextLine().split("\\s+");
                graph.addAdjacency(splitStr[0], splitStr[1]);
            }
            s2.close();
        } catch (FileNotFoundException e2) {
            System.out.print("e2: File not found");
        }
    }

    //calculates distance between two GCS coordinates using Haversine method
    //altitude variables el1/el2 ignored, returns distance in meters
    //source: https://stackoverflow.com/questions/3694380/calculating-distance-between-two-points-using-latitude-longitude/3694410#3694410
    public static double distance(double lat1, double lat2, double lon1,
                                  double lon2, double el1, double el2) {
        final int R = 6371; // Radius of the earth
        double latDistance = Math.toRadians(lat2 - lat1);
        double lonDistance = Math.toRadians(lon2 - lon1);
        double a = Math.sin(latDistance / 2) * Math.sin(latDistance / 2)
                + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2))
                * Math.sin(lonDistance / 2) * Math.sin(lonDistance / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        double distance = R * c * 1000; // convert to meters
        double height = el1 - el2;
        distance = Math.pow(distance, 2) + Math.pow(height, 2);
        return Math.sqrt(distance);
    }

    public static void main(String[] args) {
        Graph graph = new Graph();
        buildGraph(graph);
        String start, target;
        Scanner userInput = new Scanner(System.in);

        while (true) {
            System.out.print("Please enter a starting location: ");
            start = userInput.nextLine();
            if (!graph.graph.containsKey(start)) {
                System.out.print("Invalid input please try again\n");
                continue;
            }
            System.out.print("Please enter a destination: ");
            target = userInput.nextLine();
            if (!graph.graph.containsKey(target)) {
                System.out.print("Invalid input please try again\n");
                continue;
            }
            break;
        }

        boolean x = true;
        while (x) {
            //print menu and get user input
            int userChoice;
            while (true) {
                System.out.println("1: Breadth-First");
                System.out.println("2: Depth-First");
                System.out.println("3: ID-DFS");
                System.out.println("4: A*");
                System.out.println("5: Best-First");
                System.out.println("6: Five search average");
                System.out.println("7: Exit\n");
                System.out.print("Please select a choice from the menu: ");
                userChoice = Integer.parseInt(userInput.nextLine());
                if (userChoice < 1 || userChoice > 7) {
                    System.out.print("Invalid input please try again\n");
                    continue;
                }
                break;
            }

            long startT, finishT;
            double dis = 0;
            switch (userChoice) {
                case 1:
                    startT = System.nanoTime();
                    if (graph.breadthFirst(start, target)) {
                        finishT = System.nanoTime();
                        System.out.println("\nPath from " + start + " to " + target + " found!");
                        LinkedList<String> reverse = new LinkedList<String>();
                        int size = graph.path.size();
                        for (int i = 0; i < size; i++) {
                            reverse.add(graph.path.pollLast());
                        }
                        System.out.println(reverse);
                        for (int i = 1; i < reverse.size(); i++) {
                            dis = dis + distance(graph.graph.get(reverse.get(i - 1)).lat, graph.graph.get(reverse.get(i)).lat,
                                    graph.graph.get(reverse.get(i - 1)).lon, graph.graph.get(reverse.get(i)).lon, 0, 0);
                        }
                        System.out.println("Distance Traveled: " + String.format("%.2f", dis) + " meters");
                        System.out.println("Time elapsed: " + (finishT - startT) + " nanoseconds\n");
                    } else {
                        System.out.println("No valid path was found.\n");
                    }
                    graph.path.clear();
                    break;
                case 2:
                    startT = System.nanoTime();
                    if (graph.depthFirst(start, target)) {
                        finishT = System.nanoTime();
                        System.out.println("\nPath from " + start + " to " + target + " found!");
                        System.out.println(graph.path);
                        for (int i = 1; i < graph.path.size(); i++) {
                            dis = dis + distance(graph.graph.get(graph.path.get(i - 1)).lat, graph.graph.get(graph.path.get(i)).lat,
                                    graph.graph.get(graph.path.get(i - 1)).lon, graph.graph.get(graph.path.get(i)).lon, 0, 0);
                        }
                        System.out.println("Distance Traveled: " + String.format("%.2f", dis) + " meters");
                        System.out.println("Time elapsed: " + (finishT - startT) + " nanoseconds\n");
                    } else {
                        System.out.println("No valid path was found.\n");
                    }
                    graph.path.clear();
                    break;
                case 3:
                    System.out.print("Please enter max search depth: ");
                    int depthMax = Integer.parseInt(userInput.nextLine());
                    startT = System.nanoTime();
                    if (graph.IDDFS(start, target, depthMax)) {
                        finishT = System.nanoTime();
                        System.out.println("\nPath from " + start + " to " + target + " found!");
                        //reverse path order from recursive IDDFS method
                        LinkedList<String> reverse = new LinkedList<String>();
                        int size = graph.path.size();
                        for (int i = 0; i < size; i++) {
                            reverse.add(graph.path.pollLast());
                        }
                        System.out.println(reverse);
                        for (int i = 1; i < reverse.size(); i++) {
                            dis = dis + distance(graph.graph.get(reverse.get(i - 1)).lat, graph.graph.get(reverse.get(i)).lat,
                                    graph.graph.get(reverse.get(i - 1)).lon, graph.graph.get(reverse.get(i)).lon, 0, 0);
                        }
                        System.out.println("Distance Traveled: " + String.format("%.2f", dis) + " meters");
                        System.out.println("Time elapsed: " + (finishT - startT) + " nanoseconds\n");
                    } else {
                        System.out.println("No valid path was found at depth " + depthMax);
                    }
                    graph.path.clear();
                    break;
                case 4:
                    startT = System.nanoTime();
                    if (graph.aStar(start, target)) {
                        finishT = System.nanoTime();
                        System.out.println("\nPath from " + start + " to " + target + " found!");
                        LinkedList<String> reverse = new LinkedList<String>();
                        int size = graph.path.size();
                        for (int i = 0; i < size; i++) {
                            reverse.add(graph.path.pollLast());
                        }
                        System.out.println(reverse);
                        for (int i = 1; i < reverse.size(); i++) {
                            dis = dis + distance(graph.graph.get(reverse.get(i - 1)).lat, graph.graph.get(reverse.get(i)).lat,
                                    graph.graph.get(reverse.get(i - 1)).lon, graph.graph.get(reverse.get(i)).lon, 0, 0);
                        }
                        System.out.println("Distance Traveled: " + String.format("%.2f", dis) + " meters");
                        System.out.println("Time elapsed: " + (finishT - startT) + " nanoseconds\n");
                    } else {
                        System.out.println("No valid path was found.\n");
                    }
                    graph.path.clear();
                    break;
                case 5:
                    startT = System.nanoTime();
                    if (graph.bestFirst(start, target)) {
                        finishT = System.nanoTime();
                        System.out.println("\nPath from " + start + " to " + target + " found!");
                        LinkedList<String> reverse = new LinkedList<String>();
                        int size = graph.path.size();
                        for (int i = 0; i < size; i++) {
                            reverse.add(graph.path.pollLast());
                        }
                        System.out.println(reverse);
                        for (int i = 1; i < reverse.size(); i++) {
                            dis = dis + distance(graph.graph.get(reverse.get(i - 1)).lat, graph.graph.get(reverse.get(i)).lat,
                                    graph.graph.get(reverse.get(i - 1)).lon, graph.graph.get(reverse.get(i)).lon, 0, 0);
                        }
                        System.out.println("Distance Traveled: " + String.format("%.2f", dis) + " meters");
                        System.out.println("Time elapsed: " + (finishT - startT) + " nanoseconds\n");
                    } else {
                        System.out.println("No valid path was found.\n");
                    }
                    graph.path.clear();
                    break;
                case 6:
                    System.out.println("\nAverage of five searches with each method\n");

                    startT = System.nanoTime();
                    for (int i = 0; i < 5; i++) {
                        graph.breadthFirst(start, target);
                        graph.path.clear();
                    }
                    finishT = System.nanoTime();
                    System.out.println("Breadth-First: " + ((finishT - startT) / 5) + " nanoseconds");

                    startT = System.nanoTime();
                    for (int i = 0; i < 5; i++) {
                        graph.depthFirst(start, target);
                        graph.path.clear();
                    }
                    finishT = System.nanoTime();
                    System.out.println("Depth-First: " + ((finishT - startT) / 5) + " nanoseconds");

                    startT = System.nanoTime();
                    for (int i = 0; i < 5; i++) {
                        graph.IDDFS(start, target, 10);
                        graph.path.clear();
                    }
                    finishT = System.nanoTime();
                    System.out.println("ID-DFS: " + ((finishT - startT) / 5) + " nanoseconds");

                    startT = System.nanoTime();
                    for (int i = 0; i < 5; i++) {
                        graph.aStar(start, target);
                        graph.path.clear();
                    }
                    finishT = System.nanoTime();
                    System.out.println("A*: " + ((finishT - startT) / 5) + " nanoseconds");

                    startT = System.nanoTime();
                    for (int i = 0; i < 5; i++) {
                        graph.bestFirst(start, target);
                        graph.path.clear();
                    }
                    finishT = System.nanoTime();
                    System.out.println("Best-First: " + ((finishT - startT) / 5) + " nanoseconds\n");
                    break;
                case 7:
                    x = false;
                    break;
            }
        }
    }
}
