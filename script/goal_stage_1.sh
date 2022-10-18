#!/bin/bash

rostopic pub /Make_Plan/goal upo_actions/MakePlanActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  global_goal:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    pose:
      position:
        x: -4.5
        y: -1.5
        z: 3.5
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 0.0"