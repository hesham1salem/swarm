
#ifndef common_H
#define common_H
#include <queue>
#include <move_base_msgs/MoveBaseAction.h>

// GetGoals Definitions 
 move_base_msgs::MoveBaseGoal goal0,goal1,goal2,other_goal,other_goal1;
  struct goal
  {
    move_base_msgs::MoveBaseGoal point;
    int priority;

    bool operator<(const goal &other) const
    {
        return this->priority < other.priority;
    }
  };
std::priority_queue<goal> goal_queue;

// Establish connection definitions 


#endif