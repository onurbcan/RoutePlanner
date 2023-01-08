#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // m_Model.FindClosestNode method is used to find the closest nodes to the starting and ending coordinates.
    // Returned nodes are assigned to start_node and end_node variables.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// CalculateHValue method implemented by using the distance method which is developed to get the distance to another node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

// Current node is expanded by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node* node : current_node->neighbors) {
        node->parent = current_node;
        node->h_value = this->CalculateHValue(node);
        node->g_value = current_node->g_value + node->distance(*current_node);
        node->visited = true;
        open_list.push_back(node);
    }
}

// Compares summations of g and h values of 2 nodes.
bool Compare(const RouteModel::Node *node1, const RouteModel::Node *node2) {
    float f1 = node1->g_value + node1->h_value;
    float f2 = node2->g_value + node2->h_value;
    return f1 > f2;
}

// Next node according to the list with summations of g and h values is removed from the list and returned.
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node *node_to_return = open_list.back();
    open_list.pop_back();
    return node_to_return;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    path_found.push_back(*current_node);
    RouteModel::Node *currentNode = current_node;
    RouteModel::Node *parentNode = current_node->parent;
    // Iteration starting from current (final) node until starting node.
    while (parentNode != nullptr) {
        path_found.push_back(*parentNode);
        // For each node in the chain, the distance from the node to its parent is added to the distance variable.
        distance += parentNode->distance(*currentNode);
        currentNode = parentNode;
        parentNode = currentNode->parent;
    }
    // The returned vector is in the order from start node to end node.
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    start_node->visited = true;
    open_list.push_back(start_node);

    // Open list is sorted and next node is obtained using NextNode method.
    while(!open_list.empty()) {
        current_node = NextNode();
        // When the search has reached the end_node, final path is calculated.
        if(current_node->distance(*end_node) == 0) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        // All of the neighbors of the current node are added to the open_list.
        AddNeighbors(current_node);
    }
}

