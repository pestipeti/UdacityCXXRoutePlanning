#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    // Find the closest start/end node for the coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// Implement the CalculateHValue method.
// Tips:
// - [ok] You can use the distance to the end_node for the h value.
// - [ok] Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

    return node->distance(*end_node);
}


// Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - [ok] Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - [ok] For each node in current_node.neighbors, set the parent, the h_value, the g_value.
// - [ok] Use CalculateHValue below to implement the h-Value calculation.
// - [ok] For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    // Add all of the unvisited neighbors to the open nodes;
    current_node->FindNeighbors();

    for (auto neighbor: current_node->neighbors) {

        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->h_value = CalculateHValue(neighbor);

        // Add the neighbor to the open nodes.
        open_list.push_back(neighbor);

        // Mark the 'neighbor' note as visited.
        neighbor->visited = true;
    }

}


// Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - [ok] Sort the open_list according to the sum of the h value and g value.
// - [ok] Create a pointer to the node in the list with the lowest sum.
// - [ok] Remove that node from the open_list.
// - [ok] Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {

    // Sort the open nodes vector by f-values (inc).
    std::sort(open_list.begin(), open_list.end(), [](const auto &node_a, const auto &node_b) {
        return (node_a->h_value + node_a->g_value) < (node_b->h_value + node_b->g_value);
    });

    // The first value is the lowest (sorted).
    RouteModel::Node *lowest_fValueNode = open_list.front();

    // Remove the node with the lowest value
    open_list.erase(open_list.begin());

    return lowest_fValueNode;
}


// Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - [ok] This method should take the current (final) node as an argument and iteratively follow the
//   chain of parents of nodes until the starting node is found.
// - [ok] For each node in the chain, add the distance from the node to its parent to the distance variable.
// - [ok] The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found = {};
    RouteModel::Node parent_node;

    while (current_node->parent != nullptr) {
        path_found.push_back(*current_node);
        parent_node = *(current_node->parent);

        // Calculate the distance between current and its parent node, and add it to the overall distance.
        distance += current_node->distance(parent_node);

        // Move to the "next" parent.
        current_node = current_node->parent;
    }

    // Add the start node to the path.
    path_found.push_back(*current_node);

    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale();

    // Reverse to order so as the start node be the first element of the vector.
    std::reverse(path_found.begin(), path_found.end());

    return path_found;
}


// Write the A* Search algorithm here.
// Tips:
// - [ok] Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - [ok] Use the NextNode() method to sort the open_list and return the next node.
// - [ok] When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - [ok] Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;

    // Initiate the open node vector with starting node.
    open_list.push_back(start_node);

    while (!open_list.empty()) {
        current_node = NextNode();

        // Check if the current node is the end node.
        if (current_node->distance(*end_node) == 0) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        } else {
            AddNeighbors(current_node);
        }

    }

}