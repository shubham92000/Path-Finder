#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node =  &m_Model.FindClosestNode(start_x , start_y);
    end_node = &m_Model.FindClosestNode(end_x , end_y);

}



float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h_value = node->distance(*end_node);
    return h_value;
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(RouteModel::Node * neigh : current_node->neighbors){
        neigh->parent = current_node;
        neigh->h_value = CalculateHValue(neigh);
        neigh->g_value = current_node->g_value+current_node->distance(*neigh);
        open_list.push_back(neigh);
        neigh->visited = true;
    }

    return;
}


bool compare(RouteModel::Node* n1 , RouteModel::Node* n2){
    float f1;
    float f2;
    f1 = n1->h_value + n1->g_value;
    f2 = n2->h_value + n2->g_value;

    return f1<f2;
}

RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(),open_list.end(),compare);
    RouteModel::Node* smallest = nullptr;
    smallest = open_list[0];

    open_list.erase(open_list.begin());
    return smallest;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(true){
        if(start_node->x==current_node->x && start_node->y==current_node->y){
            path_found.push_back(*current_node);
            break;
        }
        path_found.push_back(*current_node);
        if(current_node->parent) distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
        
    }   

    reverse(path_found.begin(),path_found.end());
    distance *= m_Model.MetricScale();
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
    open_list.push_back(start_node);
    start_node->visited = true;

    while (!open_list.empty())
    {
        current_node=NextNode();
        if(current_node->x==end_node->x && current_node->y==end_node->y){
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
        AddNeighbors(current_node);
    }
    
    return;
}
