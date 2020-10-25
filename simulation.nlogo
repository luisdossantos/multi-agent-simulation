;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;                                                            ;;;;;;;;;;
;;;;;;;;;;                  Author : Luis Dos Santos                  ;;;;;;;;;;
;;;;;;;;;;                                                            ;;;;;;;;;;
;;;;;;;;;;                       Client : Syha                        ;;;;;;;;;;
;;;;;;;;;;                                                            ;;;;;;;;;;
;;;;;;;;;;                 School : Polytech Sorbonne                 ;;;;;;;;;;
;;;;;;;;;;                        2018 / 2019                         ;;;;;;;;;;
;;;;;;;;;;                                                            ;;;;;;;;;;
;;;;;;;;;;                        Description :                       ;;;;;;;;;;
;;;;;;;;;;   Management of a fleet of robots that harvest tomatoes'   ;;;;;;;;;;
;;;;;;;;;;         clusters in order to automatize the picking        ;;;;;;;;;;
;;;;;;;;;;              of vegetable in greenhouses                   ;;;;;;;;;;
;;;;;;;;;;                                                            ;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;---   BREEDS' DECLARATION   ---;


; We create two breeds of turtles : when the number of perpendicular lanes isn't a multiple of the number of robots, we calculate the number
; of robots that need to harvest one more lane (the residual of the Euclidean division)

breed [firstrobots firstrobot] ; agents that harvest the normal number of lanes (the quotient of the Euclidean division)
breed [lastrobots lastrobot]   ; agents that have one more lane to harvest


;---   AGENTS VARIABLES' DECLARATION   ---;


turtles-own [energy                          ; battery status
  countdown_use_energy                       ; counts the use of energy whether it's the robot moving or the robot picking a cluster, so we can handle the battery
  red_clusters_harvested                     ; number of clusters harvested by each agents
  direction                                  ; if direction = true : the agent goes up
                                             ; false : agent goes down
  direction_left_right                       ; if direction_left_right = true : the agent goes right
                                             ; false : agent goes left
  initial_pos                                ; if initial_pos = true : the agent is at its initial position (position of the agent when the environment is set up)
                                             ; false : he isn't
  number_of_lanes                            ; number of lanes harvested by each agent
  index_round_trip                           ; index that is incremented when the robot harvested a lane (went up and down) and so when index = 2 - that means that the robot harvested the lane at the top (index = 1)
                                             ; and the lane at the bottom (index = 2) - the robot can move to harvest new lanes (move the x coordinate)
  counter_ticks_waited_after_picking         ; we use a counter to count the number of ticks waited by the robot after picking a ripe cluster before he can move
                                             ; (because it takes more time to pick a ripe cluster than just move forward when the cluster isn't mature)
  wait_picking_ripe_clusters                 ; if wait_picking_ripe_clusters = true : the agent is picking a ripe cluster (so he can't move for a few ticks)
                                             ; false : agent isn't picking a cluster
  number_ticks_after_picking                 ; number of ticks that a robot has to wait after picking a ripe cluster
  robot_being_charged                        ; if robot_being_charged = true : the robot is being charged in a charging area
                                             ; false : robot isn't charging its battery
  counter_recharge_battery                   ; counter to increment the percentage of the robot's battery every X ticks (ex : robot's battery is incremented when counter_recharge_battery mod X = 0)
  back_to_lane_after_charging                ; so the robot can start at the lanes he stopped before he went charging itself
                                             ; if back_to_lane_after_charging = true, the robot just charged itself and is going back to his position
  come_back_from_lane                        ; boolean to know if the robot comes from a lane or not (and so we can increment index_round_trip properly)
                                             ; otherwise index_round_trip is incremented if the robot comes from the charging area for exemple (and we don't want so)
  robot_detected                             ; if robot_detected = true : there's another agent detected 1 or 2 patches at the right of our agent
                                             ; false : no robot detected
  avoiding_collision                         ; index so we know if the robot is currently avoiding another robot
  number_of_reloads                          ; number of times that a robot reloaded its battery
  bool_first_time_in_procedure_guide_agent   ; boolean to know if this is the first that we're in the guide-agent-to-specific-lane procedure (to set up the direction variable)
  bool_end_of_procedure_guide_agent          ; so the master knows when the agent has harvested the lane specified on the guide-agent-to-specific-lane procedure and so the master will be able to send new orders
  battery_after_finishing_his_lanes          ; percentage of the battery of the agent after he finished harvesting all his attributed lanes
  number_lane_where_agent_is                 ; to be able to implement a master, we need to know in which lane the robot is working on (simply put : the position of the robot on the greenhouse)
]


;---   PATCHES VARIABLES' DECLARATION   ---;


patches-own [countdown_patch                 ; countdown to count the number of ticks that a tomato cluster has been ripe
  limit_explosion_tomatoes                   ; limit of ticks beyond which the ripe tomatoes' clusters that haven't been harvested yet will explode (counter starts when the cluster becomes ripe / patch becomes red)
                                             ; (patches become white)
  parameter_grow_red_clusters                ; parameter : every "parameter_grow_red_clusters" ticks, some clusters become ripe (some green patches become red) (when ticks mod parameter_grow_red_clusters = 0)
]


;---   GLOBAL VARIABLES' DECLARATION   ---;


globals [height_of_central_lane              ; height of the central lane, here it's 2 patches high
  number_lanes_each_robot                    ; "normal" number of lanes that each robot has to harvest (lanes that the firstrobots have to harvest)
                                             ; it's the quotient of the Euclidean division between the total number_of_perpendicular_lanes and the number_of_robots
  number_robots_with_one_more_lane           ; number of robots that have to do one more lane than the firstrobots
  lim_pxcor                                  ; number of patches on the "x" axis (it's like world-width)
  lim_pycor                                  ; number of patches on the "y" axis (it's like world-height)
  start_pxcor_non_accessible_zone            ; x coordinate of the first recharge area
  first_terminal_occupied                    ; if first_terminal_occupied = true : the first charging station is occupied by a robot
                                             ; false : there's no robot on the charging station
  second_terminal_occupied                   ; if second_terminal_occupied = true : the second charging station is occupied by a robot
                                             ; false : there's no robot on the charging station
  robot_s_with_min_energy                    ; agentset of the robot with min energy if there's only one below the limit of energy or the 2 robots with min energy if there's two or more, so they can go recharge
  calcul_robots_min_battery_done             ; so we know if we already calculated the agentset of the robots with the minimum energy or not (because if we do the calcul every tick, the robots chosen can change)
  index_robots_in_charging_agentset_charged  ; index to know the number of robots of the robot_s_with_min_energy agentset that have been recharged
  work_with_agentset_of_robots_charging      ; we can't work with the agentset until it's defined
                                             ; (because robot_s_with_min_energy is variable, a list of agents and not really an agentset so we can't use the function "count robot_s_with_min_energy" at the beginning if robot_s_with_min_energy isn't defined)
]


;---   SETUP PROCEDURE   ---;


to setup
  clear-all ;resets the world to an initial, empty state
  setup-patches
  setup-turtles
  setup-begin-patches
  reset-ticks ;starts the tick counter
end


;---   SETUP-PATCHES PROCEDURE   ---;

; DESCRIPTION : Generate the patches

to setup-patches

  ; sizing of the environment
  set height_of_central_lane 2
  set lim_pxcor (3 * number_of_perpendicular_lanes + number_of_loading_zones) ;3*40+2 = 122
  set lim_pycor (2 * length_of_rows + height_of_central_lane)                 ;2*80+2 = 162
  set start_pxcor_non_accessible_zone (lim_pxcor - number_of_loading_zones)   ;122-2 = 120

  ; setup the patches color
  ask patches [

    ; non accessible areas
    if (pxcor >= start_pxcor_non_accessible_zone and pycor >= length_of_rows + height_of_central_lane) or (pxcor >= start_pxcor_non_accessible_zone and pycor < length_of_rows) [
      set pcolor black
    ]

    ; charging stations
    if (pycor = (length_of_rows - 1) and pxcor >= start_pxcor_non_accessible_zone) [
      set pcolor magenta
    ]

    ; central lane
    if (pycor = length_of_rows) or (pycor = (length_of_rows + height_of_central_lane - 1)) [
      set pcolor grey
    ]

    ; perpendicular lanes
    if (pycor >= length_of_rows + height_of_central_lane and pxcor < start_pxcor_non_accessible_zone and pxcor mod 3 = 1) or (pycor < length_of_rows and pxcor < start_pxcor_non_accessible_zone and pxcor mod 3 = 1) [
      set pcolor grey
    ]

    ; lanes of tomatoes' clusters
    if (pycor >= length_of_rows + height_of_central_lane and pxcor < start_pxcor_non_accessible_zone and pxcor mod 3 != 1) or (pycor < length_of_rows and pxcor < start_pxcor_non_accessible_zone and pxcor mod 3 != 1) [
      set pcolor green
    ]

    ; ripe tomatoes
    if pcolor = green [

      if ( pxcor < int ((world-width - 2) / 2) and pycor > 3 * (int (world-height / 4)) ) [
        if (random 2 < 1) [
          set pcolor red
        ]
      ]

      if ( pxcor < int ((world-width - 2) / 2) and (pycor <= 3 * (int (world-height / 4)) and pycor > 2 * (int (world-height / 4))) ) [
        if (random  3 < 1) [
          set pcolor red
        ]
      ]

      if ( pxcor < int ((world-width - 2) / 2) and (pycor <= 2 * (int (world-height / 4)) and pycor > int (world-height / 4)) ) [
        if (random 3 < 1) [
          set pcolor red
        ]
      ]

      if ( pxcor < int ((world-width - 2) / 2) and pycor <= int (world-height / 4) ) [
        if (random 5 < 1) [
          set pcolor red
        ]
      ]

      if ( pxcor >= int ((world-width - 2) / 2) and pycor > 3 * (int (world-height / 4)) ) [
        if (random 12 < 1) [
          set pcolor red
        ]
      ]

      if ( pxcor >= int ((world-width - 2) / 2) and (pycor <= 3 * (int (world-height / 4)) and pycor > 2 * (int (world-height / 4))) ) [
        if (random 3 < 1) [
          set pcolor red
        ]
      ]

      if ( pxcor >= int ((world-width - 2) / 2) and (pycor <= 2 * (int (world-height / 4)) and pycor > int (world-height / 4)) ) [
        if (random 2 < 1) [
          set pcolor red
        ]
      ]

      if ( pxcor >= int ((world-width - 2) / 2) and pycor <= int (world-height / 4) ) [
        if (random 5 < 1) [
          set pcolor red
        ]
      ]

    ]

    ; parameters
    set limit_explosion_tomatoes 42000
    set parameter_grow_red_clusters 3800

  ]
end


;---   SETUP-TURTLES PROCEDURE   ---;

; DESCRIPTION : Generate the agents

to setup-turtles

  ; gloabal variables
  set calcul_robots_min_battery_done false
  set index_robots_in_charging_agentset_charged 0
  set work_with_agentset_of_robots_charging false

  set number_lanes_each_robot int (number_of_perpendicular_lanes / number_of_robots)
  set number_robots_with_one_more_lane (number_of_perpendicular_lanes - number_lanes_each_robot * number_of_robots)

  ; firstrobots breed
  create-firstrobots (number_of_robots - number_robots_with_one_more_lane) [
    set color yellow

    ask turtle-set sort-on [who] firstrobots [
      setxy (1 + (number_lanes_each_robot * 3)*(who)) (world-height / 2)
    ]
    set heading 0
    set shape "default"

    ;global variables initialisation
    set first_terminal_occupied false
    set second_terminal_occupied false

    ;turtle variables initialisation
    set energy 100
    set countdown_use_energy 0
    set direction true
    set direction_left_right true
    set red_clusters_harvested 0
    set number_of_lanes 0
    set index_round_trip 0
    set initial_pos true
    set number_ticks_after_picking 3 ;number of ticks waited when picking a ripe cluster
    set robot_being_charged false
    set counter_recharge_battery 0

    set wait_picking_ripe_clusters false
    set counter_ticks_waited_after_picking 0

    set back_to_lane_after_charging false
    set come_back_from_lane true

    set robot_detected false
    set avoiding_collision 0

    set number_lane_where_agent_is (who * (number_lanes_each_robot * 2)) + 1

    ;variables related to the guide-agent-to-specific-lane procedure
    set bool_first_time_in_procedure_guide_agent true
    set bool_end_of_procedure_guide_agent false

    ; output variables
    set number_of_reloads 0

  ]

  ; lastrobots breed
  create-lastrobots (number_robots_with_one_more_lane) [
    set color yellow

    ask turtle-set sort-on [who] lastrobots [
      setxy (1 + (number_lanes_each_robot * 3)*(count firstrobots) + (who - count firstrobots)*(number_lanes_each_robot + 1)* 3) (world-height / 2)
    ]
    set heading 0
    set shape "default"

    ;global variables initialisation
    set first_terminal_occupied false
    set second_terminal_occupied false

    ;turtle variables initialisation
    set energy 100
    set countdown_use_energy 0
    set direction true
    set direction_left_right true
    set red_clusters_harvested 0
    set number_of_lanes 0
    set index_round_trip 0
    set initial_pos true
    set number_ticks_after_picking 3 ;number of ticks waited when picking a ripe cluster
    set robot_being_charged false
    set counter_recharge_battery 0

    set wait_picking_ripe_clusters false
    set counter_ticks_waited_after_picking 0

    set back_to_lane_after_charging false
    set come_back_from_lane true

    set robot_detected false
    set avoiding_collision 0

    set number_lane_where_agent_is ( (count firstrobots * (number_lanes_each_robot * 2) + (who - count firstrobots) * (number_lanes_each_robot + 1) * 2 ) + 1 )

    ;variables related to the guide-agent-to-specific-lane procedure
    set bool_first_time_in_procedure_guide_agent true
    set bool_end_of_procedure_guide_agent false

    ; output variables
    set number_of_reloads 0

  ]
end


;---   SETUP-BEGIN-PATCHES PROCEDURE   ---;

; DESCRIPTION : Generate cyan patches on the environment so we know where the first lane of each robots is

to setup-begin-patches
  ask patches [
    let listA (range (count firstrobots))                    ;from 0 to "count firstrobots"
    let listB (range (count firstrobots) (number_of_robots)) ;from "count firstrobots" to number_of_robots

    ; for firstrobots
    ifelse (pxcor < count firstrobots * number_lanes_each_robot * 3 - 1) [
      foreach listA [ x ->
        if ( (pycor = length_of_rows and pxcor = 1 + (number_lanes_each_robot * 3) * x) or (pycor = (length_of_rows + 1) and pxcor = 1 + (number_lanes_each_robot * 3) * x) ) [
          set pcolor cyan
        ]
      ]
    ] [
      ; for lastrobots (robots with one more lane to harvest)
      foreach listB  [ x ->
        if ( (pycor = length_of_rows and pxcor = 1 + (number_lanes_each_robot * 3)*(count firstrobots) + (x - count firstrobots)*(number_lanes_each_robot + 1)* 3) or (pycor = (length_of_rows + 1) and pxcor = 1 + (number_lanes_each_robot * 3)*(count firstrobots) + (x - count firstrobots)*(number_lanes_each_robot + 1)* 3) ) [
          set pcolor cyan
        ]
      ]
    ]
  ]
end


;---   GO PROCEDURE   ---;


to go
  grow-red-clusters
  move-turtles
  rotting-of-tomatoes
  pick-red-clusters
  ;output
  tick ;increments the tick counter
end


;---   GROW-RED-CLUSTERS PROCEDURE   ---;

; DESCRIPTION : Every "parameter_grow_red_clusters" ticks, some clusters become ripe (some green patches become red).

to grow-red-clusters
  ask patches [
    if pcolor = green [
      if (ticks mod parameter_grow_red_clusters = 0 and ticks != 0) [

        if ( pxcor < int ((world-width - 2) / 2) and pycor > 3 * (int (world-height / 4)) ) [
          if (random 2 < 1) [
            set pcolor red
          ]
        ]

        if ( pxcor < int ((world-width - 2) / 2) and (pycor <= 3 * (int (world-height / 4)) and pycor > 2 * (int (world-height / 4))) ) [
          if (random  3 < 1) [
            set pcolor red
          ]
        ]

        if ( pxcor < int ((world-width - 2) / 2) and (pycor <= 2 * (int (world-height / 4)) and pycor > int (world-height / 4)) ) [
          if (random 3 < 1) [
            set pcolor red
          ]
        ]

        if ( pxcor < int ((world-width - 2) / 2) and pycor <= int (world-height / 4) ) [
          if (random 5 < 1) [
            set pcolor red
          ]
        ]

        if ( pxcor >= int ((world-width - 2) / 2) and pycor > 3 * (int (world-height / 4)) ) [
          if (random 12 < 1) [
            set pcolor red
          ]
        ]

        if ( pxcor >= int ((world-width - 2) / 2) and (pycor <= 3 * (int (world-height / 4)) and pycor > 2 * (int (world-height / 4))) ) [
          if (random 3 < 1) [
            set pcolor red
          ]
        ]

        if ( pxcor >= int ((world-width - 2) / 2) and (pycor <= 2 * (int (world-height / 4)) and pycor > int (world-height / 4)) ) [
          if (random 2 < 1) [
            set pcolor red
          ]
        ]

        if ( pxcor >= int ((world-width - 2) / 2) and pycor <= int (world-height / 4) ) [
          if (random 5 < 1) [
            set pcolor red
          ]
        ]

      ]
    ]
  ]
end


;---   MOVE-TURTLES PROCEDURE   ---;

; DESCRIPTION : Detection of other agents around ours, and call to different procedures based on the result of the detection

to move-turtles
  ask turtles [

    ;we can't use patch-at when the robot is at the border of the map and so the patch doesn't exist
    ifelse (pycor < world-height - 2 and pycor > 1 and pxcor < world-width - 2) [

      ;if there's a turtle at right of ours
      ifelse ( ((any? turtles-on patch-at 1 0) = true or (any? turtles-on patch-at 2 0) = true or avoiding_collision != 0) and (pycor = length_of_rows or pycor = length_of_rows + 1 ) ) [
        move1
      ] [
        ;turtle at the top of ours
        ifelse ((any? turtles-on patch-at 0 1) = true or (any? turtles-on patch-at 0 2) = true)[
          move2
        ] [
          ;turtle at the bottom of ours
          ifelse ((any? turtles-on patch-at 0 -1) = true or (any? turtles-on patch-at 0 -2) = true)[
            ;if the agent is already currently avoiding another agent or not
            ifelse (avoiding_collision != 0)[
              detect-collision
            ][
              move3
            ]
          ] [
            handling-battery-turtles
          ]
        ]
      ]

    ] [
      handling-battery-turtles
    ]

    ;ON : shows the battery status of the agents & OFF : nothing
    ifelse show-energy? [ set label energy ] [ set label "" ]

  ]
end


;---   MOVE1 PROCEDURE   ---;

; DESCRIPTION : Prints a message and calls the detect-collision procedure to avoid the collision

to move1
  print "Turtle at right"

  if (heading = 90 or avoiding_collision != 0)[
    detect-collision
  ]
end


;---   MOVE2 PROCEDURE   ---;

; DESCRIPTION : If the agent is on the central lane, he moves forward (1 patch). If the agent is in a perpendicular lane, he waits that the other agent moves

to move2
  print "Turtle at the top"

  if (heading = 90 or heading = 270) [
    forward 1
  ]

  ;else : our agent is in a perpendicular lane and detected a robot on the central lane --> our agent waits that the other turtle moves
end


;---   MOVE3 PROCEDURE   ---;

; DESCRIPTION : If the agent is on the central lane, he moves forward (1 patch). If the agent is in a perpendicular lane, he waits that the other agent moves

to move3
  print "Turtle at the bottom"

  if (heading = 90 or heading = 270) [
    forward 1
  ]

  ;else : our agent is in a perpendicular lane and detected a robot on the central lane --> our agent waits that the other turtle moves
end


;---   DETECT-COLLISION PROCEDURE   ---;

; DESCRIPTION : Handles the movements of the agent to avoid a collision when both agents are on the central lane and one goes right and the other left : the agent that goes left has the
;               priority, so he continues to go left and the agent that initially goes right goes up 1 patch, right 1 patch and down 1 patch to avoid the other robot

to detect-collision
  set robot_detected true

  ;agent goes up
  ifelse (avoiding_collision = 0)[
    set heading 0
    forward 1
    set avoiding_collision (avoiding_collision + 1)
    print "turtle go up"
  ] [

    ;agent goes right
    ifelse (avoiding_collision = 1)[
    set heading 90
    forward 1
    set avoiding_collision (avoiding_collision + 1)
    print "turtle go right"
    ] [

      ;agent goes down
      if (avoiding_collision = 2)[
        set heading 180
        forward 1
        print "turtle go down"
        set avoiding_collision 0
        set robot_detected false
      ]
    ]

  ]
end


;---   HANDLING-BATTERY-TURTLES PROCEDURE   ---;

; DESCRIPTION : Main function that handles the battery status of the agents : updates the variables related to the battery (if the agent is charging its battery, if he's done charging, decrements the energy, etc.)
;               If the agent needs to recharge, calls a procedure to know if he can go recharge or not.
;               If everything is ok with the battery, calls a procedure to handle the movements of the agent
;               If the agent just completely recharged its battery, calls a function so he can go back to harvest his lanes

to handling-battery-turtles

  ;decrements the energy (the number "100" can of course be modified, it represents the speed at which the battery discharges)
  if (countdown_use_energy != 0 and countdown_use_energy mod 100 = 0 and wait_picking_ripe_clusters = false and robot_being_charged = false)[
    set energy (energy - 1)
  ]

  ;we don't need to make all this verifications while a new agentset robot_s_with_min_energy isn't calculated
  if (work_with_agentset_of_robots_charging = true) [

    ;if there's only 1 agent in the robot_s_with_min_energy agentset
    if (count robot_s_with_min_energy = 1 and member? turtle who robot_s_with_min_energy = true)[
      ;robot is charged
      if (energy = 100) [
        set calcul_robots_min_battery_done false ;we can calculate again the robots with min energy
        set work_with_agentset_of_robots_charging false ;we don't need to make all this verifications while another agentset isn't calculated
      ]
    ]

    ;if there's 2 agents in the robot_s_with_min_energy agentset
    if (count robot_s_with_min_energy = 2 and member? turtle who robot_s_with_min_energy = true)[
      ;robot is charged
      if (energy = 100)[
        ;we tell the variable that an agent of the agentset is charged
        set index_robots_in_charging_agentset_charged (index_robots_in_charging_agentset_charged + 1)
      ]
      ;if both robots of the agentset are charged, we can calculate again the robots with min energy
      if (index_robots_in_charging_agentset_charged = 2)[
        set index_robots_in_charging_agentset_charged 0
        set calcul_robots_min_battery_done false
        set work_with_agentset_of_robots_charging false ;we don't need to make all this verifications while another agentset isn't calculated
      ]
    ]
  ]

  ;if the agent is charging its battery
  ifelse (robot_being_charged = true)[
    ;if the battery isn't completely charged yet
    ifelse (energy < 100)[
      ;the number "20" can of course be modified : it represents the speed at which the battery recharges
      ifelse (counter_recharge_battery mod 20 = 0 and counter_recharge_battery != 0)[
        set energy (energy + 1)
        set counter_recharge_battery (counter_recharge_battery + 1)
      ]
      [
        set counter_recharge_battery (counter_recharge_battery + 1)
      ]
    ] [
      ;if the robot just completely charged its battery
      set robot_being_charged false
      set back_to_lane_after_charging true
      set counter_recharge_battery 0
      ;set direction true
      set heading 0
      forward 1
      set countdown_use_energy (countdown_use_energy + 1)
      set number_of_reloads (number_of_reloads + 1)
      move-back-to-last-position-before-charging
    ]
  ]

  ;The robot isn't charging its battery, he can pick clusters
  [
    ;if his battery status is under the limit, calls a function to check if he can go to the charging stations or not
    ifelse (energy <= limit_energy_for_recharge) [
      go-recharge?
    ] [
      ;if his battery is ok, the agent picks clusters
      move-turtles-normal-cycle
    ]
  ]
end


;---   MOVE-TURTLES-NORMAL-CYCLE PROCEDURE   ---;

; DESCRIPTION : Calls different functions depending on the position of the agent, but all of them have the goal to send the agents harvest clusters
;               If the robot just charged its battery, we send him to the last lane he harvested before going to the charging stations
;               If the robot harvested all his assigned lanes, we send him at his initial position to start again harvesting clusters
;               Otherwise, we call the "pick-entire-lane-and-change-lane" procedure so the robot harvests his top and bottom lane and moves right to repeat this action

to move-turtles-normal-cycle

  ;if the robot is picking a cluster, we wait (because it takes time pick a cluster)
  ifelse (wait_picking_ripe_clusters = true) [
    set counter_ticks_waited_after_picking (counter_ticks_waited_after_picking + 1)
    if (counter_ticks_waited_after_picking = number_ticks_after_picking)[
      set counter_ticks_waited_after_picking 0
      set wait_picking_ripe_clusters false
    ]
  ]

  [
    ;firstrobots breed
    ifelse (who < count firstrobots) [
      ;if the robot just charged itself, he's going to start where he stopped
      ifelse (back_to_lane_after_charging = true) [
        move-back-to-last-position-before-charging
      ] [
        ;if the robot harvested all his lanes, he is going back to his initial position to start again
        ifelse (initial_pos = false) [
          move-back-to-initial-position
        ] [
          ;if the robot just finished harvesting all his lanes
          ifelse (number_of_lanes = (number_lanes_each_robot * 4)) [
            set number_of_lanes 0
            set direction_left_right false
            set battery_after_finishing_his_lanes energy
            move-back-to-initial-position
          ] [
            ;otherwise, the agent continues harvesting
            pick-entire-lane-and-change-lane
          ]
        ]
      ]
    ]

    ;lastrobots breed -> robots with one more lane to harvest
    [
      ;if the robot just charged itself, he's going to start where he stopped
      ifelse (back_to_lane_after_charging = true) [
        move-back-to-last-position-before-charging
      ] [
        ;if the robot harvested all his lanes, he is going back to his initial position to start again
        ifelse (initial_pos = false) [
          move-back-to-initial-position
        ] [
          ;if the robot just finished harvesting all his lanes
          ifelse (number_of_lanes = ((number_lanes_each_robot + 1) * 4)) [
            set number_of_lanes 0
            set direction_left_right false
            set battery_after_finishing_his_lanes energy
            move-back-to-initial-position
          ] [
            ;otherwise, the agent continues harvesting
            pick-entire-lane-and-change-lane
          ]
        ]
      ]
    ]

  ]
end


;---   MOVE-BACK-TO-INITIAL-POSITION PROCEDURE   ---;

; DESCRIPTION : The agent moves left until he reaches its initial position

to move-back-to-initial-position

  ;firstrobots breed
  ifelse (who < count firstrobots) [
    ;while the robot isn't at its initial position
    ifelse (pxcor != (1 + (number_lanes_each_robot * 3)*(who))) [
      set heading 270
      forward 1
      set countdown_use_energy (countdown_use_energy + 1)
      set initial_pos false
    ] [
      ;agent is at its initial position
      set initial_pos true
      set index_round_trip 0
      set number_of_lanes 0
      set number_lane_where_agent_is (who * (number_lanes_each_robot * 2)) + 1
    ]
  ] [
    ;lastrobots breed

    ;while the robot isn't at its initial position
    ifelse (pxcor != (1 + (number_lanes_each_robot * 3)*(count firstrobots) + (who - count firstrobots)*(number_lanes_each_robot + 1)* 3)) [
      set heading 270
      forward 1
      set countdown_use_energy (countdown_use_energy + 1)
      set initial_pos false
    ] [
      ;agent is at its initial position
      set initial_pos true
      set index_round_trip 0
      set number_of_lanes 0
      set number_lane_where_agent_is ( (count firstrobots * (number_lanes_each_robot * 2) + (who - count firstrobots) * (number_lanes_each_robot + 1) * 2 ) + 1 )
    ]
  ]
end


;---   MOVE-BACK-TO-LAST-POSITION-BEFORE-CHARGING PROCEDURE   ---;

; DESCRIPTION : The agent moves left until he reaches its last position before he went charging its battery, so he can start harvesting clusters where he stopped

to move-back-to-last-position-before-charging

  ;firstrobots breed
  ifelse (member? turtle (who) firstrobots = true)[
    ;while the robot isn't at its initial position + the lanes he already harvested
    ifelse (pxcor != 1 + (number_lanes_each_robot * 3)*(who) + 3 * (int (number_of_lanes / 4) mod number_lanes_each_robot)) [
      set heading 270
      forward 1
      set countdown_use_energy (countdown_use_energy + 1)
      set initial_pos false
    ] [
      ;agent is at its last position before he went charging its battery
      set initial_pos true
      set back_to_lane_after_charging false
      set number_lane_where_agent_is (number_lane_where_agent_is + 1)
    ]
  ] [
    ;lastrobots breed

    ;while the robot isn't at its initial position + the lanes he already harvested
    ifelse (pxcor != 1 + (number_lanes_each_robot * 3)*(count firstrobots) + (who - count firstrobots)*(number_lanes_each_robot + 1)* 3 + 3 * (int (number_of_lanes / 4) mod (number_lanes_each_robot + 1))) [
      set heading 270
      forward 1
      set countdown_use_energy (countdown_use_energy + 1)
      set initial_pos false
    ] [
      ;agent is at its last position before he went charging its battery
      set initial_pos true
      set back_to_lane_after_charging false
      set number_lane_where_agent_is (number_lane_where_agent_is + 1)
    ]
  ]
end


;---   PICK-ENTIRE-LANE-AND-CHANGE-LANE PROCEDURE   ---;

; DESCRIPTION : The agent harvests the top and the bottom lanes of tomatoes and when it's done, moves right of 3 patches to be able to start harvesting again

to pick-entire-lane-and-change-lane

  ;we first check if the robot harvested the top and bottom lanes and so he has to move right and change lanes (x coordinate)
  ifelse (index_round_trip = 2) [
    set heading 90
    forward 3
    set countdown_use_energy (countdown_use_energy + 3)
    set index_round_trip 0

    ;we only want to increment the "number_lane_where_agent_is" variable once when the robot moves
    if (index_round_trip = 2) [
      set number_lane_where_agent_is (number_lane_where_agent_is + 1)
    ]

  ] [
    ;agent harvests top and bottom lanes

    ;we check if the robot goes through the central lane
    if (pycor = length_of_rows and come_back_from_lane = true) [
      set index_round_trip (index_round_trip + 1)
      set number_of_lanes (number_of_lanes + 1)
      set number_lane_where_agent_is (number_lane_where_agent_is + 1)
    ]

    ;agent goes up until he reaches the top limit of the greenhouse
    ifelse (direction = true) [
      ifelse (pycor < world-height - 1) [
        set heading 0
        right random 0
        forward 1
        set countdown_use_energy (countdown_use_energy + 1)
      ] [
        ;top limit is reached : robot needs to turn back
        set direction false
        set number_of_lanes (number_of_lanes + 1)
      ]
    ] [

      ;agent goes down until he reaches the bottom limit of the greenhouse
      ifelse (pycor > 0) [
        set heading 180
        right random 0
        forward 1
        set countdown_use_energy (countdown_use_energy + 1)
      ] [
        ;bottom limit is reached : robot needs to turn back
        set direction true
        set number_of_lanes (number_of_lanes + 1)
      ]
    ]

  ]

  set come_back_from_lane true
end


;---   GO-RECHARGE? PROCEDURE   ---;

; DESCRIPTION : Calculates the "robot_s_with_min_energy" agentset : the robot(s) that will recharge his (their) battery(ies).
;                     Method : if there are 1 or 2 agents with the minimum energy, the algorithm picks them
;                              if there are more than 2 the selection is random between the robots that have the same lowest value of energy
;               Also, if a robot has been selected and his going to the recharge area but is still on a perpendicular lane, he continues to harvest tomatoes while going to the central lane

to go-recharge?

  ;when the robot is still on a perpendicular lane but is going to recharge his battery, he still picks red clusters while going to the central lane
  ;so he has to wait a few ticks when he picks ripe clusters, as the usual
  ;(we don't run through the adequate procedure after this bite of code so it has to be done here)
  ifelse (calcul_robots_min_battery_done = true) [
    ifelse (member? turtle who robot_s_with_min_energy = true and energy <= limit_energy_for_recharge) [

      ifelse (wait_picking_ripe_clusters = true) [
        set counter_ticks_waited_after_picking (counter_ticks_waited_after_picking + 1)
        if (counter_ticks_waited_after_picking = number_ticks_after_picking)[
          set counter_ticks_waited_after_picking 0
          set wait_picking_ripe_clusters false
        ]
      ] [
        move-to-recharge-area
      ]

    ] [
      ;if the agent isn't part of the robots that go recharge, he continues to harvest
      move-turtles-normal-cycle
    ]
  ] [
    ;we calculate the "robot_s_with_min_energy" agentset : the robot(s) that will recharge his (their) battery(ies)

    ;if both recharge terminals are empty (no robots charging)
    ifelse (first_terminal_occupied = false and second_terminal_occupied = false and calcul_robots_min_battery_done = false) [
      set robot_s_with_min_energy min-n-of number_of_loading_zones turtles [ energy ] ;agentset of the 2 robots with min energy
      set work_with_agentset_of_robots_charging true                                  ;agentset is now defined so we can work with it
      set calcul_robots_min_battery_done true

      ifelse (member? turtle who robot_s_with_min_energy = true and energy <= limit_energy_for_recharge) [
        move-to-recharge-area
      ] [
        move-turtles-normal-cycle
      ]
    ] [

      ;if only one recharge terminal is empty
      if ( (first_terminal_occupied = true and second_terminal_occupied = false and calcul_robots_min_battery_done = false) or (first_terminal_occupied = false and second_terminal_occupied = true and calcul_robots_min_battery_done = false) ) [
        set robot_s_with_min_energy min-one-of turtles [ energy ] ;id of the robot with min energy
        set work_with_agentset_of_robots_charging true            ;agentset is now defined so we can work with it
        set calcul_robots_min_battery_done true

        ifelse (who = robot_s_with_min_energy) [
          move-to-recharge-area
        ] [
          move-turtles-normal-cycle
        ]
      ]
    ]
  ]
end


;---   MOVE-TO-RECHARGE-AREA PROCEDURE   ---;

; DESCRIPTION : The robot harvests his lane until he reaches the central lane.
;               When he does so, he moves right to the x coordinate of the 1st charging station, just above this one (y+1).
;               The agent checks if there's already a robot in the 1st station : if so he goes to the 2nd, if not, he goes to the first

to move-to-recharge-area

  ;we work with the "index_round_trip" turtle variable because if the robot harvested the top lane and goes recharging its battery, we want him to harvest the bottom lane when he comes back (and not both top and botom)
  if (pycor = length_of_rows and come_back_from_lane = true)[
    set number_of_lanes (number_of_lanes + 1)

    ifelse (index_round_trip = 0) [
     set index_round_trip 1
    ] [
      set index_round_trip 0
    ]
  ]

  set come_back_from_lane false

  ;if the agent is on the central lane
  ifelse (pycor = length_of_rows or robot_detected = true) [
    ;agent moves right to the x coordinate of the first charging station
    ifelse (pxcor != world-width - 2) [
      set heading 90
      forward 1
    ] [
      ;agent is on the x coordinate of the first charging station (just above)

      ;if there's no robot in the first charging station
      ifelse (not any? turtles-on patch-at 0 -1) [
        set heading 180
        forward 1
        set robot_being_charged true
      ] [
        ;if there's a turtle on the first charging station, agent goes to the second
        forward 1
        set heading 180
        forward 1
        set robot_being_charged true
      ]
    ]
  ] [
    ;the agent continues to harvest his lane until he reaches the central lane
    pick-entire-lane-and-change-lane
  ]
end


;---   ROTTING-OF-TOMATOES PROCEDURE   ---;

; DESCRIPTION : Increments the "countdown_patch" patch variable so we know for how long the cluster is ripe
;               And if it reaches the limit, so the cluster hasn't been harvested in a long time, the cluster explode (patch become white)

to rotting-of-tomatoes
  ask patches [
    if pcolor = red [
      set countdown_patch (countdown_patch + 1)
    ]
    if countdown_patch = limit_explosion_tomatoes [
      set pcolor white
    ]
  ]
end


;---   PICK-RED-CLUSTERS PROCEDURE   ---;

; DESCRIPTION : The agents always pick clusters at their right : so we check the patch at right of the agent (so right when he goes up, and left when he goes down)
;               and if there's a ripe cluster he harvests it and waits a few ticks while harvesting
;               If this is an exploded cluster, he still harvests it (and so waits a few ticks) so new clusters can grow but of course it doesn't count as a ripe cluster

to pick-red-clusters
  ask turtles [

    ;local variables : we use them in an "ask patch" part of this procedure to then update a turtle variable at the end of this procedure
    let test_picking_ripe_clusters false ;agent is picking a cluster
    let test_cluster_harvested false     ;agent just picked a cluster

    ;if the robot isn't charging
    if(robot_being_charged = false) [

      ;if agent is going up
      ifelse direction = true [
        ;we check the patch at right of the agent
        ask patch-at 1 0 [
          ;ripe cluster
          if (pcolor = red) [
            set pcolor green
            set countdown_patch 0
            set test_cluster_harvested true
            set test_picking_ripe_clusters true
          ]

          ;exploded cluster
          if (pcolor = white) [
            set pcolor green
            set countdown_patch 0
            set test_picking_ripe_clusters true
          ]
        ]
      ] [
        ;agent is going down

        ;we check the patch at left of the agent (from our point of view)
        ask patch-at -1 0 [
          if (pcolor = red) [
            ;ripe cluster
            set pcolor green
            set countdown_patch 0
            set test_cluster_harvested true
            set test_picking_ripe_clusters true
          ]

          ;exploded cluster
          if (pcolor = white) [
            set pcolor green
            set countdown_patch 0
            set test_picking_ripe_clusters true
          ]
        ]
      ]

      ;agent just picked a ripe cluster, we increment the "red_clusters_harvested" turtle variable : number of clusters harvested by each agent
      if (test_cluster_harvested = true)[
        set red_clusters_harvested (red_clusters_harvested + 1)
        set test_cluster_harvested false
      ]

      ;agent is picking a ripe cluster, he needs to wait
      if (test_picking_ripe_clusters = true)[
        set wait_picking_ripe_clusters true
        set test_picking_ripe_clusters false
      ]

    ]
  ]
end


;---   GUIDE-AGENT-TO-SPECIFIC-LANE PROCEDURE   ---;

; DESCRIPTION : Procedure that allows a master to send orders to a specific agent so it goes pick clusters in the lane passed as an argument

; ARGUMENTS : #agent_ID is the id of the robot to whom the master gives the order
;                   #agent_ID must be defined between 1 and number_of_robots
;                   Careful : the first agent has the "0" id on NetLogo, but in this procedure the first agent needs to be specified as "1"

;             #number_lane is the number of the lane that the agent will harvest
;                   #number_lane must be defined between 1 and number_of_perpendicular_lanes

;             #top_or_bottom is to specify if the agent will harvest the top or the bottom of the lane selected
;                   #top_or_bottom must be defined as "top" or "bottom"

to guide-agent-to-specific-lane [#agent_ID #number_lane #top_or_bottom]

  ;if arguments are incorrect
  if ( #agent_ID > number_of_robots or #agent_ID = 0 or #number_lane > number_of_perpendicular_lanes or #number_lane = 0 or (#top_or_bottom != "true" and #top_or_bottom != "false") ) [
    error "Arguments for the guide-agent-to-specific-lane procedure are incorrect \nCorrect form is : guide-agent-to-specific-lane agent_ID number_of_the_lane 'top'_or_'bottom' \n"
  ]

  ;only the selected agent goes to the lane
  if (who = #agent_ID - 1) [

    ;set the "direction" turtle variable of the agent only the first time we enter the procedure
    if (#top_or_bottom = "top" and bool_first_time_in_procedure_guide_agent = true) [
      set direction true ;agent will go up
      set bool_first_time_in_procedure_guide_agent false
    ]
    if (#top_or_bottom = "bottom" and bool_first_time_in_procedure_guide_agent = true) [
      set direction false ;agent will go down
      set bool_first_time_in_procedure_guide_agent false
    ]

    ;agent moves right
    ifelse (xcor < 1 + (3 * (#number_lane - 1)) ) [
      set heading 90
      forward 1
      set countdown_use_energy (countdown_use_energy + 1)
    ] [

      ;agent moves left
      ifelse (xcor > 1 + (3 * (#number_lane - 1)) ) [
        set heading 270
        forward 1
        set countdown_use_energy (countdown_use_energy + 1)
      ] [
        ;the robot is on the right x-coordinate of the perpendicular lane

        ;if the robot goes up
        ifelse (direction = true)[
          ;if the agent is at the top of the greenhouse, he needs to turn back
          ifelse (ycor = world-height - 1) [
            set direction false
          ] [
            ;else : agent continues to go up and harvests
            set heading 0
            forward 1
            set countdown_use_energy (countdown_use_energy + 1)
          ]
        ] [
          ;the robot goes down

          ;if the agent is at the bottom of the greenhouse, he needs to turn back
          ifelse (ycor = 0) [
            set direction true
          ] [
            ;else : agent continues to go down and harvests
            set heading 180
            forward 1
            set countdown_use_energy (countdown_use_energy + 1)
          ]
        ]

        ; we check if the agent has harvested all the lane
        if ( (ycor = length_of_rows + 2 and direction = false) or (ycor = length_of_rows - 1 and direction = true) ) [
          set bool_end_of_procedure_guide_agent true
        ]

      ]
    ]

    ; update the "number_lane_where_agent_is" variable
    if (#top_or_bottom = "top") [
      set number_lane_where_agent_is (2 * #number_lane - 1)
    ]

    if (#top_or_bottom = "bottom") [
      set number_lane_where_agent_is (2 * #number_lane)
    ]

  ]
end


;to output
; clear-output
;  output-print "Number of times that the agent reloaded its battery :"
;  ask turtles [
;    output-type "agent n°"
;    output-type who
;    output-type " : "
;    output-print number_of_reloads
;  ]
;end
@#$#@#$#@
GRAPHICS-WINDOW
422
10
907
652
-1
-1
3.91
1
10
1
1
1
0
0
0
1
0
121
0
161
1
1
1
ticks
120.0

BUTTON
189
70
252
103
NIL
setup
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
189
111
252
144
NIL
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
0

INPUTBOX
135
498
315
558
number_of_loading_zones
2.0
1
0
Number

INPUTBOX
135
268
314
328
length_of_rows
80.0
1
0
Number

INPUTBOX
135
346
313
406
number_of_perpendicular_lanes
40.0
1
0
Number

INPUTBOX
135
422
314
482
number_of_robots
7.0
1
0
Number

MONITOR
1229
460
1359
509
Agent id 0
[ number_of_lanes ] of turtle 0
17
1
12

MONITOR
1023
460
1215
509
Tous les agents
sort [ number_of_lanes ] of turtles
17
1
12

SWITCH
154
159
288
192
show-energy?
show-energy?
0
1
-1000

INPUTBOX
135
574
314
634
limit_energy_for_recharge
90.0
1
0
Number

MONITOR
1022
334
1224
383
Tous les agents
sort [ red_clusters_harvested ] of turtles
17
1
12

MONITOR
1240
335
1355
384
Agent id 0
[ red_clusters_harvested ] of turtle 0
17
1
12

MONITOR
1021
225
1214
274
Tous les agents
sort [ number_of_reloads ] of turtles
17
1
12

MONITOR
1230
225
1358
274
Agent id 0
[ number_of_reloads ] of turtle 0
17
1
12

TEXTBOX
1022
306
1307
336
Nombre de grappes de tomates mûres récoltées :
12
0.0
1

TEXTBOX
1022
196
1260
217
Nombre de cycles de recharges effectués :
12
0.0
1

TEXTBOX
1025
417
1447
477
Nombre de rangs de tomates traîtés (reset quand le robot a traité toutes les rangées attribuées) :
12
0.0
1

MONITOR
1024
572
1248
621
Tous les agents
[ robot_detected ] of turtles
17
1
12

MONITOR
1261
573
1359
622
Agent id 0
[ robot_detected ] of turtle 0
17
1
12

TEXTBOX
1025
544
1205
562
Un autre agent a été détecté :
12
0.0
1

TEXTBOX
1152
34
1261
52
** OUTPUTS **
14
0.0
1

TEXTBOX
183
235
270
253
** INPUTS **
14
0.0
1

TEXTBOX
173
28
276
46
** SETTINGS **
14
0.0
1

TEXTBOX
38
275
128
293
Liés à la serre :
12
0.0
1

TEXTBOX
33
428
127
446
Liés aux robots :
12
0.0
1

MONITOR
1022
113
1213
162
Tous les agents
sort [ battery_after_finishing_his_lanes ] of turtles
17
1
12

TEXTBOX
1023
85
1445
105
Pourcentage de la batterie après avoir traîté toutes les rangées attribuées :
12
0.0
1

MONITOR
1225
114
1353
163
Agent id 0
[ battery_after_finishing_his_lanes ] of turtle 0
17
1
12

MONITOR
1024
621
1300
666
NIL
sort [ number_lane_where_agent_is ] of turtles
17
1
11

@#$#@#$#@
## WHAT IS IT?

(a general understanding of what the model is trying to show or explain)

## HOW IT WORKS

(what rules the agents use to create the overall behavior of the model)

## HOW TO USE IT

(how to use the model, including a description of each of the items in the Interface tab)

## THINGS TO NOTICE

(suggested things for the user to notice while running the model)

## THINGS TO TRY

(suggested things for the user to try to do (move sliders, switches, etc.) with the model)

## EXTENDING THE MODEL

(suggested things to add or change in the Code tab to make the model more complicated, detailed, accurate, etc.)

## NETLOGO FEATURES

(interesting or unusual features of NetLogo that the model uses, particularly in the Code tab; or where workarounds were needed for missing features)

## RELATED MODELS

(models in the NetLogo Models Library and elsewhere which are of related interest)

## CREDITS AND REFERENCES

(a reference to the model's URL on the web if it has one, as well as any other necessary credits, citations, and links)
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

sheep
false
15
Circle -1 true true 203 65 88
Circle -1 true true 70 65 162
Circle -1 true true 150 105 120
Polygon -7500403 true false 218 120 240 165 255 165 278 120
Circle -7500403 true false 214 72 67
Rectangle -1 true true 164 223 179 298
Polygon -1 true true 45 285 30 285 30 240 15 195 45 210
Circle -1 true true 3 83 150
Rectangle -1 true true 65 221 80 296
Polygon -1 true true 195 285 210 285 210 240 240 210 195 210
Polygon -7500403 true false 276 85 285 105 302 99 294 83
Polygon -7500403 true false 219 85 210 105 193 99 201 83

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

wolf
false
0
Polygon -16777216 true false 253 133 245 131 245 133
Polygon -7500403 true true 2 194 13 197 30 191 38 193 38 205 20 226 20 257 27 265 38 266 40 260 31 253 31 230 60 206 68 198 75 209 66 228 65 243 82 261 84 268 100 267 103 261 77 239 79 231 100 207 98 196 119 201 143 202 160 195 166 210 172 213 173 238 167 251 160 248 154 265 169 264 178 247 186 240 198 260 200 271 217 271 219 262 207 258 195 230 192 198 210 184 227 164 242 144 259 145 284 151 277 141 293 140 299 134 297 127 273 119 270 105
Polygon -7500403 true true -1 195 14 180 36 166 40 153 53 140 82 131 134 133 159 126 188 115 227 108 236 102 238 98 268 86 269 92 281 87 269 103 269 113

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270
@#$#@#$#@
NetLogo 6.0.4
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180
@#$#@#$#@
0
@#$#@#$#@
