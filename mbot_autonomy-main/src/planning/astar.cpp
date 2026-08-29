#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                             mbot_lcm_msgs::pose2D_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    bool found_path = false;
    
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    Node* startNode = new Node(startCell.x, startCell.y);
    Node* goalNode = new Node(goalCell.x, goalCell.y);
    std::priority_queue<Node*, std::vector<Node*>, Compare_Node> Q;
    std::vector<Node*> elements;
    startNode->g_cost = 0.0;
    startNode->h_cost = h_cost(startNode,goalNode,distances); 
    Q.push(startNode);
    while (!Q.empty())
    {
        Node *curr = Q.top();
        Q.pop();
        if (*curr == *goalNode)
        {
            std::cout << "path found";
            break;
        }
        elements = expand_node(curr,distances,params);
        for (auto element : elements )
        {
            double tentative_gCost = curr->g_cost + g_cost(curr, element, distances, params);
            if ( tentative_gCost < g_cost(startNode,element,distances,params))
            {
                element->g_cost = tentative_gCost;
                element->h_cost = h_cost(element, goalNode, distances);
                element->parent = curr;
                Q.push(element);   
            }   
        }
    }
    
    



    

    mbot_lcm_msgs::path2D_t path;
    path.utime = start.utime;
    if (found_path)
    {
        auto nodePath = extract_node_path(goalNode, startNode);
        path.path = extract_pose_path(nodePath, distances);
        // Remove last pose, and add the goal pose
        path.path.pop_back();
        path.path.push_back(goal);
    }

    else printf("[A*] Didn't find a path\n");
    path.path_length = path.path.size();
    return path;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    double h_cost = 0.0;
    ////////////////// TODO: Implement your heuristic //////////////////////////
    double x = std::abs(from->cell.x - goal->cell.x);
    double y = std::abs(from->cell.y - goal->cell.y);
    h_cost = std::sqrt(x*x + y*y);
    return h_cost;
}
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    double g_cost = 0.0;
    ////////////////// TODO: Implement your goal cost, use obstacle distances //////////////////////////
    double x = std::abs(from->cell.x - goal->cell.x);
    double y = std::abs(from->cell.y - goal->cell.y);
    g_cost = std::sqrt(x*x + y*y);
    double cost = 0.0;

    double goal_ob_dis = distances(goal->cell.x,goal->cell.y);
    
    if (goal_ob_dis < params.maxDistanceWithCost && goal_ob_dis > params.minDistanceToObstacle)
    {
        cost = std::pow(params.maxDistanceWithCost - goal_ob_dis,params.distanceCostExponent);
    } 

    double from_ob_dis = distances(from->cell.x,from->cell.y);
    if (from_ob_dis < params.maxDistanceWithCost && from_ob_dis > params.minDistanceToObstacle)
    {
        cost += std::pow(params.maxDistanceWithCost - from_ob_dis, params.distanceCostExponent);
    }
    g_cost = g_cost + cost;


    
    
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    std::vector<Node*> children;
    ////////////////// TODO: Implement your expand node algorithm //////////////////////////
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};
    for (int n = 0; n < 8; ++n)
    {
        Node adjacentCell(node->cell.x + xDeltas[n], node->cell.y + yDeltas[n]);
        if (distances.isCellInGrid(adjacentCell.cell.x, adjacentCell.cell.y))
        {
            auto Distance = distances(adjacentCell.cell.x,adjacentCell.cell.y);
            if (Distance > params.minDistanceToObstacle)
            {
                Node *newNode = new Node(adjacentCell.cell.x,adjacentCell.cell.y);
                children.push_back(newNode);
            }
            
        }
    }

    
    
    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    std::vector<Node*> path;
    
    ////////////////// TODO: Implement your extract node function //////////////////////////
    // Traverse nodes and add parent nodes to the vector
    
    // Reverse path
    
    Node *temp_node = goal_node;
    while (goal_node)
    {
       path.push_back(temp_node);
       temp_node = temp_node->parent;
       if (*temp_node == *start_node)
       {
        path.push_back(start_node);
        break;
       }
    }


    std::reverse(path.begin(), path.end());
    return path;
}
// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    std::vector<mbot_lcm_msgs::pose2D_t> path;
    ////////////////// TODO: Implement your extract_pose_path function //////////////////////////
    // This should turn the node path into a vector of poses (with heading) in the global frame
    // You should prune the path to get a waypoint path suitable for sending to motion controller
    mbot_lcm_msgs::pose2D_t point;
    std::vector<Node*> prune_nodes = prune_node_path(nodes);

    for (int i = 0;i<prune_nodes.size();i++)
    {  
        
        auto current_pose = grid_position_to_global_position(prune_nodes[i]->cell,distances);
        point.x = current_pose.x;
        point.y = current_pose.y;
        path.push_back(point);       
        
    }
   



    
    
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;

}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath)
{
    std::vector<Node*> new_node_path;
    ////////////////// TODO: Optionally implement a prune_node_path function //////////////////////////
    // This should remove points in the path along the same line
    if (nodePath.size() < 3)
    {
        return nodePath;
    }
    new_node_path.push_back(nodePath[0]);
    for (int i = 1;i< nodePath.size();i++)
    {
      Node *prev = nodePath[i-1];
      Node *curr = nodePath[i];
      Node *next = nodePath[i+1];
      double cross_dist = (curr->cell.y - prev->cell.y)*(next->cell.x - curr->cell.x) -
                          (next->cell.y - curr->cell.y)*(curr->cell.x - prev->cell.x);
      
      if (cross_dist > 1e-1)
      {
        new_node_path.push_back(curr);
      }

    }
    new_node_path.push_back(nodePath[nodePath.size()-1]);

    
    return new_node_path;

}
