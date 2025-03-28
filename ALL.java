//MERGESORT
import java.util.Scanner;

public class MergeSort {

    private static int[] b; // Auxiliary array for merging

    public static void mergeSort(int[] a, int low, int high) {
        if (low < high) {
            int mid = (low + high) / 2;
            mergeSort(a, low, mid);
            mergeSort(a, mid + 1, high);
            merge(a, low, mid, high);
        }
    }

    public static void merge(int[] a, int low, int mid, int high) {
        int i = low, j = mid + 1, k = low;

        while (i <= mid && j <= high) {
            if (a[i] <= a[j]) {
                b[k++] = a[i++];
            } else {
                b[k++] = a[j++];
            }
        }

        // Copy remaining elements from the left subset
        while (i <= mid) {
            b[k++] = a[i++];
        }

        // Copy remaining elements from the right subset
        while (j <= high) {
            b[k++] = a[j++];
        }

        // Copy the merged elements back into the original array
        for (int h = low; h <= high; h++) {
            a[h] = b[h];
        }
    }

    // Main method to test MergeSort
    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        System.out.println("Enter the number of elements in the array:");
        int n = scanner.nextInt();
        int[] array = new int[n];
        b = new int[n]; // Initialize the auxiliary array

        System.out.println("Enter the elements of the array:");
        for (int i = 0; i < n; i++) {
            array[i] = scanner.nextInt();
        }

        mergeSort(array, 0, array.length - 1);

        System.out.println("Sorted Array:");
        for (int i = 0; i < array.length; i++) {
                 System.out.print(array[i] + " ");
}
        scanner.close();
    }
}
/*Enter the number of elements in the array:
3
Enter the elements of the array:
5
3
6
Sorted Array:
3 5 6 */








//QUICKSORT
import java.util.Scanner;

public class QuickSort {

    public static int partition(int[] a, int low, int high) {
        int pivot = a[low];
        int i = low + 1;
        int j = high;

        while (i <= j) {
            while (i <= j && a[i] <= pivot)
                i++;

            while (i <= j && a[j] >= pivot)
                j--;

            if (i < j)
                interchange(a, i, j);
        }
        interchange(a, low, j);
        return j;
    }

    public static void interchange(int[] a, int i, int j) {
        int temp = a[i];
        a[i] = a[j];
        a[j] = temp;
    }

    public static void quickSort(int[] a, int low, int high) {
        if (low < high) {
            int j = partition(a, low, high);
            quickSort(a, low, j - 1);
            quickSort(a, j + 1, high);
        }
    }

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        System.out.println("Enter the number of elements in the array:");
        int n = scanner.nextInt();
        int[] array = new int[n];

        System.out.println("Enter the elements of the array:");
        for (int i = 0; i < n; i++) {
            array[i] = scanner.nextInt();
        }

        quickSort(array, 0, array.length - 1);

        System.out.println("Sorted Array:");
        for (int num : array) {
            System.out.print(num + " ");
        }

        scanner.close();
    }
}
/*Enter the number of elements in the array:
3
Enter the elements of the array:
3

6
1
Sorted Array:
1 3 6 */           
 











//BFS_AM
import java.util.*;

class BFS_AM {
    static void bfs(int[][] adj, int start) {
        boolean[] visited = new boolean[5];
        List<Integer> q = new ArrayList<>();
        q.add(start);
        visited[start] = true;
        int vis;
        while (!q.isEmpty()) {
            vis = q.get(0);
            System.out.print(vis + " ");
            q.remove(q.get(0));
            for (int i = 0; i < 5; i++) {
                if (adj[vis][i] == 1 && (!visited[i])) {
                    q.add(i);
                    visited[i] = true;
                }
            }
        }
    }

    public static void main(String[] args) {
        int adj[][] = { { 0, 1, 1, 0, 0 }, { 1, 0, 1, 0, 0 }, { 0, 1, 0, 1, 1 },
                { 0, 0, 1, 0, 0 }, { 0, 0, 1, 0, 0 } };
        bfs(adj, 0);
    }
}

//DFS_AM
import java.util.*;

public class DFSUsingAM {
    public static void dfs(int[][] adjacencyMatrix, boolean[] visited, int vertex) {
        visited[vertex] = true;
        System.out.print(vertex + " ");

        for (int i = 0; i < adjacencyMatrix.length; i++) {
            if (adjacencyMatrix[vertex][i] == 1 && !visited[i]) {
                dfs(adjacencyMatrix, visited, i);
            }
        }
    }

    public static void main(String[] args) {
        int[][] adjacencyMatrix = {
            {0, 1, 1, 0, 0},
            {1, 0, 0, 1, 1},
            {1, 0, 0, 0, 1},
            {0, 1, 0, 0, 1},
            {0, 1, 1, 1, 0}
        };

        boolean[] visited = new boolean[adjacencyMatrix.length];
        System.out.println("DFS Traversal:");
        dfs(adjacencyMatrix, visited, 0);
    }
}






//BFS_AL

import java.util.*;

public class BFSUsingAL {
    public static void bfs(int vertices, List<List<Integer>> adjacencyList, int start) {
        boolean[] visited = new boolean[vertices];
        Queue<Integer> queue = new LinkedList<>();

        visited[start] = true;
        queue.add(start);

        System.out.println("BFS Traversal:");
        while (!queue.isEmpty()) {
            int vertex = queue.poll();
            System.out.print(vertex + " ");

            for (int neighbor : adjacencyList.get(vertex)) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    queue.add(neighbor);
                }
            }
        }
    }

    public static void main(String[] args) {
        int vertices = 5;
        List<List<Integer>> adjacencyList = new ArrayList<>();
        
        for (int i = 0; i < vertices; i++) {
            adjacencyList.add(new ArrayList<>());
        }

        adjacencyList.get(0).add(1);
        adjacencyList.get(0).add(2);
        adjacencyList.get(1).add(3);
        adjacencyList.get(1).add(4);
        adjacencyList.get(2).add(4);

        bfs(vertices, adjacencyList, 0);
    }
}
list bfs

//DFS_AL
import java.util.*;

public class DFSUsingAL {
    public static void dfs(List<List<Integer>> adjacencyList, boolean[] visited, int vertex) {
        visited[vertex] = true;
        System.out.print(vertex + " ");

        for (int neighbor : adjacencyList.get(vertex)) {
            if (!visited[neighbor]) {
                dfs(adjacencyList, visited, neighbor);
            }
        }
    }

    public static void main(String[] args) {
        int vertices = 5;
        List<List<Integer>> adjacencyList = new ArrayList<>();

        for (int i = 0; i < vertices; i++) {
            adjacencyList.add(new ArrayList<>());
        }

        adjacencyList.get(0).add(1);
        adjacencyList.get(0).add(2);
        adjacencyList.get(1).add(3);
        adjacencyList.get(1).add(4);
        adjacencyList.get(2).add(4);

        boolean[] visited = new boolean[vertices];
        System.out.println("DFS Traversal:");
        dfs(adjacencyList, visited, 0);
    }
}
 


//MAX_HEAP
import java.util.Arrays;
public class MaxHeap {
    private int[] heap;
    private int size;
    private int capacity;
    // Constructor to initialize the heap
    public MaxHeap(int capacity) {
        this.capacity = capacity;
        this.size = 0;
        heap = new int[capacity];
    }
    // Method to get the index of the parent node
    private int parent(int index) {
        return (index - 1) / 2;
    }
    // Method to get the index of the left child
    private int leftChild(int index) {
        return 2 * index + 1;
    }
    // Method to get the index of the right child
    private int rightChild(int index) {
        return 2 * index + 2;
    }
    // Method to heapify (maintain heap property)
    private void heapify(int index) {
        int left = leftChild(index);
        int right = rightChild(index);
        int largest = index;

        // Check if left child is larger than the current node
        if (left < size && heap[left] > heap[largest]) {
            largest = left;
        }

        // Check if right child is larger than the current largest
        if (right < size && heap[right] > heap[largest]) {
            largest = right;
        }

        // If largest is not the current node, swap and recursively heapify
        if (largest != index) {
            swap(index, largest);
            heapify(largest);
        }
    }

    // Method to swap two elements in the heap
    private void swap(int i, int j) {
        int temp = heap[i];
        heap[i] = heap[j];
        heap[j] = temp;
    }

    // Method to insert an element into the heap
    public void insert(int value) {
        if (size == capacity) {
            System.out.println("Heap is full");
            return;
        }

        // Insert the new element at the end
        heap[size] = value;
        int current = size;
        size++;

        // Fix the heap property by bubbling up
        while (current > 0 && heap[current] > heap[parent(current)]) {
            swap(current, parent(current));
            current = parent(current);
        }
    }

    // Method to extract the maximum element from the heap
    public int extractMax() {
        if (size == 0) {
            System.out.println("Heap is empty");
            return Integer.MIN_VALUE; // or an appropriate error code
        }

        // The root contains the maximum element
        int max = heap[0];

        // Replace the root with the last element
        heap[0] = heap[size - 1];
        size--;

        // Heapify the root element
        heapify(0);

        return max;
    }

    // Method to get the maximum element without extracting it
    public int getMax() {
        if (size == 0) {
            System.out.println("Heap is empty");
            return Integer.MIN_VALUE;
        }
        return heap[0];
    }

    // Method to print the heap elements
    public void printHeap() {
        System.out.println(Arrays.toString(Arrays.copyOf(heap, size)));
    }

    public static void main(String[] args) {
        MaxHeap maxHeap = new MaxHeap(10);

        maxHeap.insert(10);
        maxHeap.insert(20);
        maxHeap.insert(5);
        maxHeap.insert(30);
        maxHeap.insert(15);

        System.out.println("Heap after insertions:");
        maxHeap.printHeap();

        System.out.println("Extracted Max: " + maxHeap.extractMax());

        System.out.println("Heap after extraction:");
        maxHeap.printHeap();

        System.out.println("Current Max: " + maxHeap.getMax());
    } Explain this code line by line please explain clearly 
}
/*Heap after insertions:
[30, 20, 5, 10, 15]
Extracted Max: 30
Heap after extraction:
[20, 15, 5, 10]
Current Max: 20*/






//MIN_HEAP
import java.util.Arrays;

public class MinHeap {
    private int[] heap;
    private int size;
    private int capacity;

    // Constructor to initialize the heap
    public MinHeap(int capacity) {
        this.capacity = capacity;
        this.size = 0;
        heap = new int[capacity];
    }

    // Method to get the index of the parent node
    private int parent(int index) {
        return (index - 1) / 2;
    }

    // Method to get the index of the left child
    private int leftChild(int index) {
        return 2 * index + 1;
    }

    // Method to get the index of the right child
    private int rightChild(int index) {
        return 2 * index + 2;
    }

    // Method to maintain min-heap property
    private void heapify(int index) {
        int left = leftChild(index);
        int right = rightChild(index);
        int smallest = index;

        // Check if left child is smaller than the current node
        if (left < size && heap[left] < heap[smallest]) {
            smallest = left;
        }

        // Check if right child is smaller than the current smallest
        if (right < size && heap[right] < heap[smallest]) {
            smallest = right;
        }

        // If smallest is not the current node, swap and recursively heapify
        if (smallest != index) {
            swap(index, smallest);
            heapify(smallest);
        }
    }

    // Method to swap two elements in the heap
    private void swap(int i, int j) {
        int temp = heap[i];
        heap[i] = heap[j];
        heap[j] = temp;
    }

    // Method to insert an element into the heap
    public void insert(int value) {
        if (size == capacity) {
            System.out.println("Heap is full");
            return;
        }

        // Insert the new element at the end
        heap[size] = value;
        int current = size;
        size++;

        // Fix the heap property by bubbling up
        while (current > 0 && heap[current] < heap[parent(current)]) {
            swap(current, parent(current));
            current = parent(current);
        }
    }

    // Method to extract the minimum element from the heap
    public int extractMin() {
        if (size == 0) {
            System.out.println("Heap is empty");
            return Integer.MAX_VALUE;
        }

        // The root contains the minimum element
        int min = heap[0];

        // Replace the root with the last element
        heap[0] = heap[size - 1];
        size--;

        // Heapify the root element
        heapify(0);

        return min;
    }

    // Method to get the minimum element without extracting it
    public int getMin() {
        if (size == 0) {
            System.out.println("Heap is empty");
            return Integer.MAX_VALUE;
        }
        return heap[0];
    }

    // Method to print the heap elements
    public void printHeap() {
        System.out.println(Arrays.toString(Arrays.copyOf(heap, size)));
    }

    // Main method to test MinHeap
    public static void main(String[] args) {
        MinHeap minHeap = new MinHeap(10);

        minHeap.insert(10);
        minHeap.insert(20);
        minHeap.insert(5);
        minHeap.insert(30);
        minHeap.insert(15);

        System.out.println("Heap after insertions:");
        minHeap.printHeap();

        System.out.println("Extracted Min: " + minHeap.extractMin());

        System.out.println("Heap after extraction:");
        minHeap.printHeap();

        System.out.println("Current Min: " + minHeap.getMin());
    }
}
/*Heap after insertions:
[5, 15, 10, 30, 20]
Extracted Min: 5
Heap after extraction:
[10, 15, 20, 30]
Current Min: 10*/






//JOB_DEADLINE

import java.util.*;

class GfG {

    static ArrayList<Integer> jobSequencing(int[] id, 
            int[] deadline, int[] profit) {
        int n = id.length;
        ArrayList<Integer> ans = new ArrayList<>(Arrays.asList(0, 0));

        // pair the profit and deadline of
        // all the jobs together
        ArrayList<int[]> jobs = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            jobs.add(new int[]{profit[i], deadline[i]});
        }

        // sort the jobs based on profit
        // in decreasing order
        jobs.sort((a, b) -> b[0] - a[0]);

        // array to store result of job sequence
        int[] result = new int[n];
        Arrays.fill(result, -1);

        for (int i = 0; i < n; i++) {
            int start = Math.min(n, jobs.get(i)[1]) - 1;
            for (int j = start; j >= 0; j--) {

                // if slot is empty
                if (result[j] == -1) {
                    result[j] = i;
                    break;
                }
            }
        }

        for (int i = 0; i < n; i++) {
            if (result[i] != -1) {
                ans.set(1, ans.get(1) + jobs.get(result[i])[0]);
                ans.set(0, ans.get(0) + 1);
            }
        }

        return ans;
    }

    public static void main(String[] args) {
        int[] id = {1, 2, 3, 4, 5};
        int[] deadline = {2, 1, 2, 1, 1};
        int[] profit = {100, 19, 27, 25, 15};
        ArrayList<Integer> ans = jobSequencing(id, deadline, profit);
        System.out.println(ans.get(0) + " " + ans.get(1));
    }
}
//2 127








//singlesourceshortestpath Matrix
import java.util.*;

class ShortestPathMatrix {
    static final int INF = Integer.MAX_VALUE;

    public static int[] dijkstra(int[][] graph, int src) {
        int V = graph.length;
        int[] dist = new int[V];
        boolean[] visited = new boolean[V];
        Arrays.fill(dist, INF);
        dist[src] = 0;
        
        for (int i = 0; i < V - 1; i++) {
            int u = minDistance(dist, visited);
            visited[u] = true;
            
            for (int v = 0; v < V; v++) {
                if (!visited[v] && graph[u][v] != 0 && dist[u] != INF && dist[u] + graph[u][v] < dist[v]) {
                    dist[v] = dist[u] + graph[u][v];
                }
            }
        }
        return dist;
    }
    
    private static int minDistance(int[] dist, boolean[] visited) {
        int min = INF, minIndex = -1;
        for (int v = 0; v < dist.length; v++) {
            if (!visited[v] && dist[v] < min) {
                min = dist[v];
                minIndex = v;
            }
        }
        return minIndex;
    }
    
    public static void main(String[] args) {
        int[][] graph = {
            {0, 4, 0, 0, 0, 0, 0, 8, 0},
            {4, 0, 8, 0, 0, 0, 0, 11, 0},
            {0, 8, 0, 7, 0, 4, 0, 0, 2},
            {0, 0, 7, 0, 9, 14, 0, 0, 0},
            {0, 0, 0, 9, 0, 10, 0, 0, 0},
            {0, 0, 4, 14, 10, 0, 2, 0, 0},
            {0, 0, 0, 0, 0, 2, 0, 1, 6},
            {8, 11, 0, 0, 0, 0, 1, 0, 7},
            {0, 0, 2, 0, 0, 0, 6, 7, 0}
        };
        int[] dist = dijkstra(graph, 0);
        System.out.println("Vertex Distance from Source");
        for (int i = 0; i < dist.length; i++) System.out.println(i + " \t " + dist[i]);
    }
}

/*Vertex Distance from Source
0 	 0
1 	 4
2 	 12
3 	 19
4 	 21
5 	 11
6 	 9
7 	 8
8 	 14*/





//singlesourceshortestpath list
import java.util.*;

class ShortestPathList {
    static class Node {
        int v, w;
        Node(int v, int w) { this.v = v; this.w = w; }
    }

    public static int[] dijkstra(int V, ArrayList<ArrayList<Node>> graph, int src) {
        int[] dist = new int[V];
        Arrays.fill(dist, Integer.MAX_VALUE);
        dist[src] = 0;
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(n -> n.w));
        pq.add(new Node(src, 0));
        
        while (!pq.isEmpty()) {
            Node curr = pq.poll();
            for (Node n : graph.get(curr.v)) {
                if (dist[curr.v] + n.w < dist[n.v]) {
                    dist[n.v] = dist[curr.v] + n.w;
                    pq.add(new Node(n.v, dist[n.v]));
                }
            }
        }
        return dist;
    }

    public static void main(String[] args) {
        int V = 9;
        ArrayList<ArrayList<Node>> graph = new ArrayList<>();
        for (int i = 0; i < V; i++) graph.add(new ArrayList<>());
        int[][] edges = { {0,1,4}, {0,7,8}, {1,2,8}, {1,7,11}, {2,3,7}, {2,8,2}, {2,5,4}, {3,4,9}, {3,5,14}, {4,5,10}, {5,6,2}, {6,7,1}, {6,8,6}, {7,8,7} };
        for (int[] e : edges) { graph.get(e[0]).add(new Node(e[1], e[2])); graph.get(e[1]).add(new Node(e[0], e[2])); }
        
        int[] dist = dijkstra(V, graph, 0);
        System.out.println("Vertex Distance from Source");
        for (int i = 0; i < V; i++) System.out.println(i + " \t " + dist[i]);
    }
}
/*Vertex Distance from Source
0 	 0
1 	 4
2 	 12
3 	 19
4 	 21
5 	 11
6 	 9
7 	 8
8 	 148*/
