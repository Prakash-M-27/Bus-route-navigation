#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#define MAX_STOPS 20
#define MAX_ROUTES 10
#define INF INT_MAX
#define MAX_NAME 50

// Structure for Bus Stop
typedef struct {
    int id;
    char name[MAX_NAME];
    float latitude;
    float longitude;
} BusStop;

// Structure for Route Edge
typedef struct {
    int dest;
    int distance;
    char routeNumber[10];
} RouteEdge;

// Structure for Graph
typedef struct {
    BusStop stops[MAX_STOPS];
    RouteEdge adjMatrix[MAX_STOPS][MAX_STOPS];
    int numStops;
} Graph;

// Structure for Path
typedef struct {
    int stops[MAX_STOPS];
    int numStops;
    int totalDistance;
} Path;

// Initialize Graph
void initGraph(Graph* g) {
    g->numStops = 0;
    for (int i = 0; i < MAX_STOPS; i++) {
        for (int j = 0; j < MAX_STOPS; j++) {
            g->adjMatrix[i][j].dest = -1;
            g->adjMatrix[i][j].distance = INF;
            strcpy(g->adjMatrix[i][j].routeNumber, "");
        }
    }
}

// Add Bus Stop
int addStop(Graph* g, const char* name, float lat, float lon) {
    if (g->numStops >= MAX_STOPS) {
        printf("Maximum stops reached!\n");
        return -1;
    }
    
    int id = g->numStops;
    g->stops[id].id = id;
    strcpy(g->stops[id].name, name);
    g->stops[id].latitude = lat;
    g->stops[id].longitude = lon;
    g->numStops++;
    
    return id;
}

// Add Route between stops
void addRoute(Graph* g, int src, int dest, int distance, const char* routeNum) {
    if (src >= 0 && src < g->numStops && dest >= 0 && dest < g->numStops) {
        g->adjMatrix[src][dest].dest = dest;
        g->adjMatrix[src][dest].distance = distance;
        strcpy(g->adjMatrix[src][dest].routeNumber, routeNum);
        
        // For bidirectional routes
        g->adjMatrix[dest][src].dest = src;
        g->adjMatrix[dest][src].distance = distance;
        strcpy(g->adjMatrix[dest][src].routeNumber, routeNum);
    }
}

// Find minimum distance vertex
int minDistance(int dist[], int visited[], int n) {
    int min = INF, minIndex = -1;
    
    for (int i = 0; i < n; i++) {
        if (!visited[i] && dist[i] < min) {
            min = dist[i];
            minIndex = i;
        }
    }
    return minIndex;
}

// Dijkstra's Algorithm for shortest path
Path findShortestPath(Graph* g, int src, int dest) {
    Path result;
    result.numStops = 0;
    result.totalDistance = INF;
    
    if (src < 0 || src >= g->numStops || dest < 0 || dest >= g->numStops) {
        return result;
    }
    
    int dist[MAX_STOPS];
    int visited[MAX_STOPS];
    int parent[MAX_STOPS];
    
    // Initialize
    for (int i = 0; i < g->numStops; i++) {
        dist[i] = INF;
        visited[i] = 0;
        parent[i] = -1;
    }
    
    dist[src] = 0;
    
    // Dijkstra's algorithm
    for (int count = 0; count < g->numStops - 1; count++) {
        int u = minDistance(dist, visited, g->numStops);
        
        if (u == -1) break;
        
        visited[u] = 1;
        
        for (int v = 0; v < g->numStops; v++) {
            if (!visited[v] && 
                g->adjMatrix[u][v].distance != INF &&
                dist[u] != INF &&
                dist[u] + g->adjMatrix[u][v].distance < dist[v]) {
                
                dist[v] = dist[u] + g->adjMatrix[u][v].distance;
                parent[v] = u;
            }
        }
    }
    
    // Reconstruct path
    if (dist[dest] == INF) {
        return result;
    }
    
    int path[MAX_STOPS];
    int pathLength = 0;
    int current = dest;
    
    while (current != -1) {
        path[pathLength++] = current;
        current = parent[current];
    }
    
    // Reverse path
    for (int i = 0; i < pathLength; i++) {
        result.stops[i] = path[pathLength - 1 - i];
    }
    
    result.numStops = pathLength;
    result.totalDistance = dist[dest];
    
    return result;
}

// Display all routes
void displayAllRoutes(Graph* g) {
    printf("\n========================================\n");
    printf("ALL BUS ROUTES\n");
    printf("========================================\n");
    
    for (int i = 0; i < g->numStops; i++) {
        for (int j = i + 1; j < g->numStops; j++) {
            if (g->adjMatrix[i][j].distance != INF) {
                printf("Route %s: %s <-> %s (Distance: %d km)\n",
                       g->adjMatrix[i][j].routeNumber,
                       g->stops[i].name,
                       g->stops[j].name,
                       g->adjMatrix[i][j].distance);
            }
        }
    }
}

// Display shortest path
void displayPath(Graph* g, Path path) {
    if (path.numStops == 0) {
        printf("\nNo route found!\n");
        return;
    }
    
    printf("\n========================================\n");
    printf("SHORTEST ROUTE FOUND\n");
    printf("========================================\n");
    printf("Total Distance: %d km\n", path.totalDistance);
    printf("Number of Stops: %d\n\n", path.numStops);
    
    printf("Route Path:\n");
    for (int i = 0; i < path.numStops; i++) {
        int stopId = path.stops[i];
        printf("%d. %s (Lat: %.4f, Lon: %.4f)\n", 
               i + 1,
               g->stops[stopId].name,
               g->stops[stopId].latitude,
               g->stops[stopId].longitude);
        
        if (i < path.numStops - 1) {
            int next = path.stops[i + 1];
            printf("   |  Bus: %s  |  Distance: %d km\n",
                   g->adjMatrix[stopId][next].routeNumber,
                   g->adjMatrix[stopId][next].distance);
            printf("   v\n");
        }
    }
}

// Display all stops
void displayAllStops(Graph* g) {
    printf("\n========================================\n");
    printf("ALL BUS STOPS\n");
    printf("========================================\n");
    for (int i = 0; i < g->numStops; i++) {
        printf("%d. %s (%.4f, %.4f)\n", 
               i,
               g->stops[i].name,
               g->stops[i].latitude,
               g->stops[i].longitude);
    }
}

// Load sample dataset (simulating OpenStreetMap data)
void loadSampleData(Graph* g) {
    printf("Loading Bus Route Dataset...\n");
    
    // Add Bus Stops (simulating real city data)
    int s0 = addStop(g, "Central Station", 11.0168, 76.9558);
    int s1 = addStop(g, "Town Hall", 11.0145, 76.9634);
    int s2 = addStop(g, "RS Puram", 11.0074, 76.9584);
    int s3 = addStop(g, "Gandhipuram", 11.0179, 76.9673);
    int s4 = addStop(g, "Singanallur", 10.9894, 77.0254);
    int s5 = addStop(g, "Peelamedu", 11.0270, 76.9864);
    int s6 = addStop(g, "Saibaba Colony", 11.0203, 76.9597);
    int s7 = addStop(g, "Hope College", 11.0031, 76.9689);
    int s8 = addStop(g, "Ukkadam", 10.9975, 76.9547);
    int s9 = addStop(g, "Airport", 11.0330, 77.0443);
    
    // Add Routes (Bus Number, Distance in km)
    addRoute(g, s0, s1, 3, "1A");
    addRoute(g, s1, s2, 2, "1A");
    addRoute(g, s2, s3, 4, "2B");
    addRoute(g, s0, s3, 5, "3C");
    addRoute(g, s3, s5, 6, "4D");
    addRoute(g, s1, s6, 3, "5E");
    addRoute(g, s6, s5, 4, "5E");
    addRoute(g, s2, s7, 3, "6F");
    addRoute(g, s7, s8, 2, "6F");
    addRoute(g, s0, s8, 4, "7G");
    addRoute(g, s4, s8, 5, "8H");
    addRoute(g, s4, s7, 4, "8H");
    addRoute(g, s5, s9, 8, "9I");
    addRoute(g, s3, s9, 12, "10J");
    
    printf("Dataset loaded successfully!\n");
    printf("Total Stops: %d\n", g->numStops);
}

int main() {
    Graph busGraph;
    initGraph(&busGraph);
    
    printf("========================================\n");
    printf("BUS ROUTE NAVIGATION SYSTEM\n");
    printf("Using Graph Data Structure\n");
    printf("SDG 11 - Sustainable Cities & Communities\n");
    printf("========================================\n\n");
    
    // Load dataset
    loadSampleData(&busGraph);
    
    // Display all stops
    displayAllStops(&busGraph);
    
    // Display all routes
    displayAllRoutes(&busGraph);
    
    // Example 1: Find shortest path from Central Station to Airport
    printf("\n\n========================================\n");
    printf("EXAMPLE 1: Central Station -> Airport\n");
    printf("========================================\n");
    Path path1 = findShortestPath(&busGraph, 0, 9);
    displayPath(&busGraph, path1);
    
    // Example 2: Find shortest path from Town Hall to Singanallur
    printf("\n\n========================================\n");
    printf("EXAMPLE 2: Town Hall -> Singanallur\n");
    printf("========================================\n");
    Path path2 = findShortestPath(&busGraph, 1, 4);
    displayPath(&busGraph, path2);
    
    // Example 3: Find shortest path from RS Puram to Peelamedu
    printf("\n\n========================================\n");
    printf("EXAMPLE 3: RS Puram -> Peelamedu\n");
    printf("========================================\n");
    Path path3 = findShortestPath(&busGraph, 2, 5);
    displayPath(&busGraph, path3);
    
    printf("\n========================================\n");
    printf("SYSTEM BENEFITS (SDG 11):\n");
    printf("========================================\n");
    printf("✓ Optimizes public transport usage\n");
    printf("✓ Reduces travel time and congestion\n");
    printf("✓ Promotes sustainable urban mobility\n");
    printf("✓ Improves accessibility for citizens\n");
    printf("✓ Reduces carbon emissions\n");
    printf("========================================\n\n");
    
    return 0;
}
