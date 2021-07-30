#include "route_planner.h"
#include <algorithm>
#include <vector>
#include <iostream>
using std::sort;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return end_node->distance(* node);
}

// Expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(RouteModel::Node *node : current_node->neighbors) {
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);
        open_list.emplace_back(node);
        node->visited = true;
    }
}

// Sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *node_a, const RouteModel::Node *node_b) {
        float f1 = node_a->h_value + node_a->g_value;
        float f2 = node_b->h_value + node_b->g_value;
        return f1 > f2;
    });
    RouteModel::Node *node = open_list.back();
    open_list.pop_back();
    return node;
}


// Return the final path found from A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while (current_node != start_node) {
        distance += current_node->distance(*current_node->parent);
        path_found.emplace(path_found.begin(), *current_node);
        current_node = current_node->parent;
    }
    path_found.emplace(path_found.begin(), *start_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* Search algorithm
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // set current node to start node
    // and add to open list as "visited" node
    current_node = start_node;
    open_list.emplace_back(current_node);
    current_node->visited = true;

    // iterate while there are still open nodes...
    while(open_list.size() > 0) {
        // ... until we reach the end, then construct final path
        if(current_node->x == end_node->x && current_node->y == end_node->y) {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        } else {
            // if not at the end, expand neighbours and move to next node
            AddNeighbors(current_node);
            current_node = NextNode();
        }
    }

}