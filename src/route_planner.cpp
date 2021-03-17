#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));

}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float dist = node->distance(*end_node);
    return dist;
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto& nei : current_node->neighbors) {
        nei->parent = current_node;
        nei->g_value = current_node->g_value + current_node->distance(*nei);
        nei->h_value = CalculateHValue(nei);
        open_list.emplace_back(nei);
        nei->visited = true;
    }
}



RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), 
    	[] (const RouteModel::Node* a, const RouteModel::Node* b) 	{
    		return a->g_value + a->h_value < b->g_value + b->h_value;
    	}
    );
    RouteModel::Node* lowest_node = open_list.front();
    open_list.erase(open_list.begin());
    return lowest_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;// what's the usage of this variable?
    std::vector<RouteModel::Node> path_found;

    while(current_node->parent != nullptr) {
        path_found.emplace_back(*current_node);
        const auto parent = *(current_node->parent);
		distance += current_node->distance(parent);
        current_node = current_node->parent;
    }
    path_found.emplace_back(*start_node);
    std::reverse(path_found.begin(),path_found.end()); 
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}



void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
  	start_node->visited = true;
  	open_list.emplace_back(start_node);
    while(!open_list.empty()) {
        auto current_node = NextNode();

        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
}