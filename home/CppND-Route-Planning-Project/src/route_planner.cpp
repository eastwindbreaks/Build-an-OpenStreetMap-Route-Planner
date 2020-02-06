#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    
    this->start_node = &(model.FindClosestNode(start_x, start_y));
    this->end_node = &(model.FindClosestNode(end_x, end_y));
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{
    return (this->end_node)->distance(*node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) 
{
    current_node->FindNeighbors();
    for (auto i : (current_node->neighbors))
    {
      i->parent = current_node;
      i->h_value = RoutePlanner::CalculateHValue(i);
      i->g_value = current_node->distance(*i) + current_node->g_value; //z注意这里相当重要，g_value 要自己计算与parent点的距离， 并且天上parent的g_value的值
      this->open_list.push_back(i);
      i->visited = true;
    }
}

bool Compare( RouteModel::Node* node1, RouteModel::Node* node2) {
  float f1 = node1->g_value + node1->h_value; // f1 = g1 + h1
  float f2 = node2->g_value + node2->h_value; // f2 = g2 + h2
  return f1 > f2; 
}
RouteModel::Node *RoutePlanner::NextNode() 
{
    sort((this->open_list).begin(), (this->open_list).end(), Compare);
    RouteModel::Node* lowest_sum = (this->open_list).back();
    (this->open_list).pop_back();
    return lowest_sum;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    path_found.push_back(*current_node);
    while(current_node->parent != nullptr)
    {
      distance = distance + current_node->distance(*(current_node->parent));
      path_found.push_back(*(current_node->parent));
      current_node = current_node->parent;
    }
    reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    current_node = this->start_node;
    current_node->visited = true;   
    while (current_node != this->end_node)
    {
      RoutePlanner::AddNeighbors(current_node);
      current_node = RoutePlanner::NextNode();   
    }          
    RoutePlanner::m_Model.path = RoutePlanner::ConstructFinalPath(current_node);   
}
