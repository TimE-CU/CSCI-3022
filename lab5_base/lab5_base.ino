/**
 * IMPORTANT: Read through the code before beginning implementation!
 * Your solution should fill in the various "TODO" items within this starter code.
 */

#include <sparki.h>
#define TRUE 1
#define FALSE 0

#define CYCLE_TIME .100

// Number of vertices to discretize the map
#define NUM_X_CELLS 4
#define NUM_Y_CELLS 4

// Map is ~60cm x 42cm
#define MAP_SIZE_X 0.6
#define MAP_SIZE_Y 0.42

#define BIG_NUMBER 9999


// Grid map from Lab 4: values of 1 indicate free space, 0 indicates empty space
bool world_map[NUM_Y_CELLS][NUM_X_CELLS];

// Destination (I,J) grid coordinates
int goal_i = 2;
int goal_j = 2;

// Start (I,J) grid coordinates
int source_i = 1;
int source_j = 1;

// Pointer to ordered sequence of waypoints
int *path = NULL;


void setup() {   
  // Dijkstra Setup -- initialize empty world map
  for (int j = 0; j < NUM_Y_CELLS; ++j) {
    for (int i = 0; i < NUM_X_CELLS; ++i) {
      world_map[j][i] = 1;
    }
  }

  //TODO: Set up your map here by setting individual cells to 0 to indicate obstacles
  world_map[0][1] = 0; // Example of setup code to indicate an obstacle at grid position (1,0)
  world_map[1][1] = 0; // Example of setup code to indicate an obstacle at grid position (1,1)
}

/*****************************
 * Dijkstra Helper Functions *
 *****************************/

// Return 1 if there are entries in range [0,inf) in arr
// otherwise return 0, signifying empty queue
bool is_empty(int *arr, int len) {
  for (int i=0; i < len; ++i) {
    if (arr[i] >= 0) {
      return TRUE;
    }
  }
  return FALSE;
}

// Return the index with the minimum value in int array "arr" of length "len"
// Assumes positive values only, with values of "-1" indicating 'empty'
int get_min_index(int *arr, int len) {
  int min_val=-1, min_idx=-1;
  for (int i=0;i < len; ++i) {
    if(arr[i] == -1)
    {
      continue;
    }
    if (arr[i] < min_val || min_val == -1) {
      min_val = arr[i];
      min_idx = i;
    }
  }
  return min_idx;
}


/**********************************
 * Coordinate Transform Functions *
 **********************************/

// Converts a vertex index into (I,J) coordinates
// Returns 0 if something went wrong -- assume invalid i and j values being set
bool vertex_index_to_ij_coordinates(int v_idx, int *i, int *j) {
  *i = v_idx % NUM_X_CELLS;
  *j = v_idx / NUM_X_CELLS;
  
  if (*i < 0 || *j < 0 || *i >= NUM_X_CELLS || *j >= NUM_Y_CELLS) return FALSE;
  return TRUE;
}

// Converts (I,J) coordinates into a vertex index
int ij_coordinates_to_vertex_index(int i, int j) {
  return j*NUM_X_CELLS + i;  
}

// Convert (i,j) coordinates into world coordinates
// Returns 0 if something went wrong -- assume invalid x and y values are being set
// Returns 1 otherwise. Assigned x and y values are the middle of cell (i,j)
bool ij_coordinates_to_xy_coordinates(int i, int j, float *x, float *y) {
  if (i < 0 || j < 0 || i >= NUM_X_CELLS || j >= NUM_Y_CELLS) return FALSE;
  
  *x = (i+0.5)*(MAP_SIZE_X/NUM_X_CELLS);
  *y = (j+0.5)*(MAP_SIZE_Y/NUM_Y_CELLS);
  return TRUE;  
}

// Convert (x,y) world coordinates into (i,j) grid coordinates
// Returns 0 if something went wrong -- assume invalid x and y values are being set
// Returns 1 otherwise. x and y values are the middle of cell (i,j)
bool xy_coordinates_to_ij_coordinates(float x, float y, int *i, int *j) {
  if (x < 0 || y < 0 || x >= MAP_SIZE_X || y >= MAP_SIZE_Y) return FALSE;
  
  *i = int((x/MAP_SIZE_X) * NUM_X_CELLS);
  *j = int((y/MAP_SIZE_Y) * NUM_Y_CELLS);

  return TRUE;  
}

/**********************************
 *      Core Dijkstra Functions   *
 **********************************/
int counter = 0;
// Returns the cost of moving from vertex_source to vertex_dest
int get_travel_cost(int vertex_source, int vertex_dest) {
  int i1, i2, j1, j2;
  vertex_index_to_ij_coordinates(vertex_source, &i1, &j1);
  vertex_index_to_ij_coordinates(vertex_dest, &i2, &j2);
  if(!world_map[i1][j1] || !world_map[i2][j2])
  {
    return BIG_NUMBER;
  }
  
  int dist = abs(i1-i2) + abs(j1-j2);
  if(dist == 0)
  {
    return 0;
  }
  if(dist == 1)
  {
    return 1;
  }
  return BIG_NUMBER;
}


// Allocate and return a list of ints corresponding to the "prev" variable in Dijkstra's algorithm
// The returned array prev can be treated as a lookup table:  prev[vertex_index] = next vertex index on the path back to source_vertex
int *run_dijkstra(int source_vertex) {
  // Array mapping vertex_index to distance of shortest path from source_vertex to vertex_index.
  int dist[NUM_X_CELLS*NUM_Y_CELLS];
  for(int i = 0; i < NUM_Y_CELLS*NUM_X_CELLS; i++)
  {
    dist[i] = BIG_NUMBER;
  }
  
  // Queue for identifying which vertices are still being explored -- Q_cost[vertex_index] = shortest known dist to get to vertex_index. 
  // Q_cost[vertex_index] = -1 if the vertex is no longer being considered.
  int Q_cost[NUM_Y_CELLS*NUM_X_CELLS]; 
  for(int i = 0; i < NUM_Y_CELLS*NUM_X_CELLS; i++)
  {
    Q_cost[i] = get_travel_cost(source_vertex, i);
  }

  // Initialize memory for prev array
  int *prev = new int[NUM_X_CELLS*NUM_Y_CELLS];
  prev[source_vertex] = -1;

  /**
   * TODO: Insert your Dijkstra's code here
   */
  // Add neighbors of source_vertex to Q_cost
  while(true)
  {
    // find min node
    int min_idx = get_min_index(Q_cost, NUM_X_CELLS*NUM_Y_CELLS);

    // Update Q_cost
    for(int i = 0; i < NUM_Y_CELLS*NUM_X_CELLS; i++)
    {
      if(Q_cost[i] == -1)
      {
        continue;
      }
      
      int new_cost = get_travel_cost(min_idx, i) + Q_cost[min_idx];
      if(new_cost < Q_cost[i])
      {
        Q_cost[i] = new_cost;
        prev[i] = min_idx;
      }
    }
    
    Q_cost[min_idx] = -1;
    if(is_empty(Q_cost, NUM_X_CELLS*NUM_Y_CELLS))
    {
      break;
    }
  }
  
  return prev;
}

// Given a populated 'prev' array, a source vertex, and destination vertex,
// allocate and return an integer array populated with the path from source to destination.
// The first entry of your path should be source_vertex and the last entry should be "-1" 
// to indicate the end of the array since paths can be variable length.
int *reconstruct_path(int *prev, int source_vertex, int dest_vertex) {
  int final_path[NUM_X_CELLS*NUM_Y_CELLS];

  /**
   * TODO: Insert your code here
   */ 
  int final_path_idx = 0;
  int prev_node = dest_vertex;
  while(true)
  {
    final_path[final_path_idx] = prev_node;
    final_path_idx++;
    prev_node = prev[prev_node];
    if(prev_node == source_vertex)
    {
      break;
    }
  }
  return final_path;
}






void loop () {
  sparki.clearLCD();
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
  int *prev = NULL;
  int *path = NULL;


  /**
   * TODO: Populate prev with dijkstra's algorithm, then populate path with reconstruct_path
   */
  int source = ij_coordinates_to_vertex_index(source_i, source_j);
  int dest = ij_coordinates_to_vertex_index(goal_i, goal_j);
  prev = run_dijkstra(source);
  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 4; j++)
    {
//      sparki.println(prev[ij_coordinates_to_vertex_index(i, j)]);/
    }
  }
  path = reconstruct_path(prev, source, dest);
  sparki.println(path[0]);
  sparki.updateLCD();
  while(true);
  if (prev != NULL) {
    delete prev; 
    prev = NULL; // Once we have the path, don't need to keep prev around in memory anymore.
  }

  sparki.clearLCD();

  // TODO
  // Display the final path in the following format:
  //
  //  Source: (0,0)
  //  Goal: (3,1)
  //  0 -> 1 -> 2 -> 6 -> 7
  int path_idx = 0;
  while(true)
  {
    sparki.print(path[path_idx]);
    sparki.print(" -> ");
    if(path[path_idx] == -1)
    {
      break;
    }
    path_idx++;
  }
  sparki.println();
  sparki.updateLCD();

  if (path != NULL) {
    delete path; 
    path=NULL; // Important! Delete the arrays returned from run_dijkstra and reconstruct_path when you're done with them!
  }
  ///////////////////////////////////////////////////  
 
  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10); // Accept some error
}
