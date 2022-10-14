#include "graph_traversal.hpp"

#include <cassert>
#include <iostream>
#include <limits>
#include <vector>

// A helper method for shortest_path that returns the index of the vertex that
// should be the next current node, given which nodes have already been visited
// and the tentative distances to nodes.
//
// The return value indicates whether the algorithm should continue running. If
// true, the next current index is returned through the reference parameter
// next_current.
bool next_current_for_shortest_path(
    const std::vector<bool>& visited, const std::vector<int>& distance,
    int& next_current) {
  // Assert the visited and distance vectors are the same size. If they are not,
  // a programming mistake has been made.
  assert(visited.size() == distance.size());

  // Initialize next_current_distance to be the (tentative) distance to some
  // (reachable) vertex that has not yet been visited.
  int next_current_distance = std::numeric_limits<int>::max();
  for (int v = 0; v < visited.size(); ++v) {
    if (!visited[v] && distance[v] < std::numeric_limits<int>::max()) {
      next_current = v;
      next_current_distance = distance[v];
      break;
    }
  }

  // Return false if next_current_distance wasn't set to something smaller than
  // std::numeric_limits<int>::max(). For Djikstra's algorithm, this happens
  // when we have visited every node reachable from the start vertex.
  if (next_current_distance == std::numeric_limits<int>::max()) {
    return false;
  }

  // Iterate through all the vertices, looking for the unvisited node with the
  // smallest (tentative) distance. As we find better candidates, update
  // next_current and next_current_distance.
  for (int v = 0; v < visited.size(); ++v) {
    if (!visited[v] && distance[v] < next_current_distance) {
      next_current = v;
      next_current_distance = distance[v];
    }
  }

  // Return true, as we have set next_current to an appropriate index.
  return true;
}

template <class Graph>
std::vector<bool> distance_at_most_two(const Graph& graph, const int start) {
  
  // Implement the distance_at_most_two method.
  //
  // There are many different valid and good strategies to implement this
  // method.
  //
  // * It is possible to modify breadth first search, keeping track of how many
  //   edges have been traversed.
  // * It is possible to compute the vertices adjacent to the start vertex and
  //   the vertices adjacent to those vertices.
  // * It is possible to use shortest_path on a different graph.

  // Throw an std::range_error exception if start is not a valid index. 
  if (start < 0 || start >= graph.vertex_count()) {
    throw std::range_error("start is not a valid index.");
  }

  // Create a vector to represent the vertices that are seen. 
  std::vector<bool> seen = std::vector<bool>(graph.vertex_count(), false);

  // Mark start as true because the distance from start is 0
  seen[start] = true;

  // for every out-vertex of start...
  for (const int i : graph.out_edges(start)) {
    //...mark i as seen. 
    // This represents vertex i being 1 edge away from start. 
    seen[i] = true;
    // for every out-vertex of i...
    for (const int j : graph.out_edges(i)) {
      //...mark vertex j as true.
      // This represents j being 2 edges away from start.
      seen[j] = true;
    }
  }
  return seen;

}

template <class Graph>
std::vector<int> shortest_path(const Graph& graph, const int start) {
  
  // Implement Djikstra's algorithm for finding the shortest paths from a given
  // vertex. For more information and as a reference, see
  // https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm.
  // 
  // START(Djikstra's Algorithm)
  //
  // PROCEDURE shortest_path(const Graph& graph, const int start)
  //
  //   Throw a std::invalid_argument exception if start is not a valid vertex.
  //
  //   Create a vector to track whether each node has been visited. Initialize
  //   it by setting all nodes as not visited.
  //
  //   Create a vector to track the shortest distance to each vertex from the
  //   start vertex. Initialize it by setting the start node as distance 0 and
  //   other nodes as distance infinity, which we represent by
  //   std::numeric_limits<int>::max().
  //
  //   Create a variable to track the current node being processed. Initialize
  //   it by setting it to be the start vertex.
  //
  //   WHILE (next_current_for_shortest_path)
  //
  //     For every out-edge of the current vertex, determine whether the path
  //     through the current vertex is shorter than the best path seen so far.
  //     If it is better, update its shortest distance.
  //
  //     Mark the current vertex as visited.
  //
  //  END WHILE
  //
  //  Return the vector of shortest distances.
  //
  // END PROCEDURE
  //
  // STOP

  // Website used for reference:
  // https://www.youtube.com/watch?v=pVfj6mxhdMw

  // Throw an std::range_error exception if start is not a valid index.
  if (start < 0 || start >= graph.vertex_count()) {
    throw std::range_error("start is not a valid index.");
  }

  // Vector of booleans to keep track of nodes visited. 
  // All the booleans are initialized to false. 
  std::vector<bool> seen = std::vector<bool>(graph.vertex_count(), false);

  // Vector to keep track of the shortest distance 
  // to each vertex from the start vertex. 
  std::vector<int> distances = std::vector<int>(graph.vertex_count(), std::numeric_limits<int>::max());

  // Initialize the start vertex to have distance 0;
  distances[start] = 0;

  // int to track the current node.
  int current_node = start;

  while (next_current_for_shortest_path(seen, distances, current_node)) {
    // for every out vertex of the current node...
    for (const int out_vertex : graph.out_edges(current_node)) {
      // ... create an int to keep track of the current node's shortest distance.
      int current_node_distance = distances[current_node];
      
      // Create int to keep track of the distance from 
      // the current node to the current out vertex. 
      int current_edge = graph.edge_weight(current_node, out_vertex);

      // If the distance of the current node plus the distance 
      // of current_edge is less than than the shortest distance of the current out vertex...
      if (current_node_distance + current_edge < distances[out_vertex]) {
        // ...update the distance of the current out vertex because a shorter path has been found.
        distances[out_vertex] = current_node_distance + current_edge;
      }
    }
    // Mark the current_node as seen.
    seen[current_node] = true;
  }

  return distances;

}

// Since the implementation of distance_at_most_two is in the cpp file, we need
// to tell the compiler which template instantiations to make.
//
// For more information, see
// https://isocpp.org/wiki/faq/templates#templates-defn-vs-decl.
template std::vector<bool> distance_at_most_two<AdjacencyListGraph>(
    const AdjacencyListGraph& graph, const int start);
template std::vector<bool> distance_at_most_two<AdjacencyMatrixGraph>(
    const AdjacencyMatrixGraph& graph, const int start);
template std::vector<bool>
distance_at_most_two<UndirectedGraph<AdjacencyListGraph>>(
    const UndirectedGraph<AdjacencyListGraph>& graph, const int start);
template std::vector<bool>
distance_at_most_two<UndirectedGraph<AdjacencyMatrixGraph>>(
    const UndirectedGraph<AdjacencyMatrixGraph>& graph, const int start);

// Since the implementation of shortest_path is in the cpp file, we need to tell
// the compiler which template instantiations to make.
//
// For more information, see
// https://isocpp.org/wiki/faq/templates#templates-defn-vs-decl.
template std::vector<int> shortest_path<AdjacencyListGraph>(
    const AdjacencyListGraph& graph, const int start);
template std::vector<int> shortest_path<AdjacencyMatrixGraph>(
    const AdjacencyMatrixGraph& graph, const int start);
template std::vector<int>
shortest_path<UndirectedGraph<AdjacencyListGraph>>(
    const UndirectedGraph<AdjacencyListGraph>& graph, const int start);
template std::vector<int>
shortest_path<UndirectedGraph<AdjacencyMatrixGraph>>(
    const UndirectedGraph<AdjacencyMatrixGraph>& graph, const int start);