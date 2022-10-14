#include "airport_network.hpp"

#include <cassert>
#include <string>
#include <utility>
#include <vector>

#include "adjacency_list_graph.hpp"
#include "adjacency_matrix_graph.hpp"
#include "airport_database.hpp"
#include "graph_traversal.hpp"
#include "undirected_graph.hpp"

AirportNetwork::AirportNetwork(const AirportDatabase& airport_database) : airport_database_(airport_database), airport_graph_(airport_database.size())

// Implement the AirportNetwork constructor.
//
// Like every constructor, this should properly set all of the private data
// members of the class. You should do as much initialization as possible in the
// initializer list, though some will need to be done in the method body.

{
  // for every FlightRoute in the database's vector of flight routes... 
  for (const FlightRoute flight : airport_database.routes()) {
    // ...create an int to store the index of the first airport.
    int i = airport_database.index(flight.code_one());

    // create an int to store the index of the second aiport.
    int j = airport_database.index(flight.code_two());

    // Create an Aiport object to store the first airport in flight.
    Airport airport_one = airport_database.airport(flight.code_one());

    // Create an Airport object to store the second airport in flight.  
    Airport airport_two = airport_database.airport(flight.code_two());

    // Find the great-circle distance between the first and second airports. 
    int great_circle_dist = airport_one.distance_miles(airport_two);

    // Add an edge from the first airport to the second airport with edge weight of
    // the great-circle distance between them. 
    airport_graph_.add_edge(i, j, great_circle_dist);
  }
  
}


int AirportNetwork::num_airports() const noexcept {
  return airport_graph_.vertex_count();
}

int AirportNetwork::num_flight_routes() const noexcept {
  return airport_graph_.edge_count();
}

std::vector<std::string> AirportNetwork::at_most_one_layover(
    const std::string& code) const {

  // Implement the at_most_one_layover function.
  //
  // HINT: This will require a modicum of business logic. The graph theory
  // part of the problem should be delegated to a method call in
  // graph_traversal.
  
  // Use AirportDatabase::index() to find the index of code; it also takes care of the
  // std::range_error exception when it is called if code is not found in the database.

  int start = airport_database_.index(code);
  
  // Store the vector of booleans that distance_at_most_two function returns.
  std::vector<bool> num_of_layovers = distance_at_most_two(airport_graph_, start);

  // Create a vector of strings that will store the codes of airports that are at
  // most one layover away from code. 
  std::vector<std::string> airport_codes;

  // for every vertex in airport_graph_...
  for (int i = 0; i < airport_graph_.vertex_count(); ++i) {
    // ...if the boolean at the ith position in num_of_layovers is true...
    if (num_of_layovers[i] == true) {
      // ...create an std::string to store the airport code of the current vertex.
      std::string current_code = airport_database_.code(i);

      // Push the current code to the vector of strings 
      airport_codes.push_back(current_code);
    }
  }

  return airport_codes;

}

std::vector<int> AirportNetwork::least_distance(const std::string& code) const {

  // Implement the least_distance function.
  //
  // HINT: This will require a small amount of business logic. The graph theory
  // part of the problem should be delegated to a method call in
  // graph_traversal.
  
  // Use AirportDatabase::index() to find the index of code; it also takes care of the
  // std::range_error exception when it is called if code is not found in the database.
  int start = airport_database_.index(code);

  // To find the shortest path distance of travel, we can use shortest_path.
  // shortest_path returns the shortest path from code to each airport. 

  std::vector<int> distances = shortest_path(airport_graph_, start);

  return distances;

}