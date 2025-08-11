extensions [csv]

;; ==========================================
;; TRAFFIC SIMULATION MODEL
;; CMPLXSY Final Project
;; Members: Adrian Yung Cheng, Krizchelle Wong, Wayne Co, Patrick Leonida
;; ==========================================

;; ==========================================
;; GLOBALS
;; ==========================================

globals [
  time-step
  total-cars
  cars-completed
  average-travel-time
  congestion-level
  traffic-light-timer
  simulation-data
  global-traffic-phase
  sum-travel-time-completed
  throughput
]

;; ==========================================
;; BREEDS
;; ==========================================

breed [cars car]
breed [traffic-lights traffic-light]

;; ==========================================
;; CAR PROPERTIES
;; ==========================================

cars-own [
  destination-x
  destination-y
  travel-time
  route
  speed
  max-speed
  waiting-time
  is-waiting
  lane
]

;; ==========================================
;; TRAFFIC LIGHT PROPERTIES
;; ==========================================

traffic-lights-own [
  state  ; "red" or "green"
  light-timer
  cycle-time
  direction  ; "horizontal" or "vertical"
]

;; ==========================================
;; PATCH PROPERTIES
;; ==========================================

patches-own [
  is-road
  is-intersection
  road-direction  ; "horizontal", "vertical", or "both"
  car-here
  sig-timer sig-threshold sig-phase  ; per-intersection signal controller
]

;; ==========================================
;; SETUP PROCEDURES
;; ==========================================

to setup
  clear-all
  reset-ticks

  ;; Initialize global variables
  set time-step 0
  set total-cars 0
  set cars-completed 0
  set average-travel-time 0
  set congestion-level 0
  set traffic-light-timer 0
  set simulation-data []
  set global-traffic-phase "vertical" ;; Default to vertical phase
  set sum-travel-time-completed 0
  set throughput 0

  ;; Setup the environment
  setup-road-grid
  setup-traffic-lights
  setup-cars

  ;; Initialize data collection
  collect-initial-data
end

;; --- Double-lane road grid ---
to setup-road-grid
  let min-x min [pxcor] of patches
  let max-x max [pxcor] of patches
  let min-y min [pycor] of patches
  let max-y max [pycor] of patches

  ;; Initialize all patches efficiently
  ask patches [
    set is-road false
    set is-intersection false
    set road-direction "none"
    set car-here false
    set pcolor black
  ]

  ;; Create road network more efficiently - use single pass
  ask patches with [
    ;; Skip 2 from edges for buffer
    pxcor > min-x + 1 and pxcor < max-x - 1 and
    pycor > min-y + 1 and pycor < max-y - 1 and
    ;; Create double-lane roads
    (pycor mod 5 = 0 or pycor mod 5 = 1 or
     pxcor mod 5 = 0 or pxcor mod 5 = 1)
  ] [
    set is-road true
    set pcolor gray

    ;; Determine road direction correctly
    if (pycor mod 5 = 0 or pycor mod 5 = 1) and (pxcor mod 5 = 0 or pxcor mod 5 = 1) [
      set is-intersection true
      set road-direction "both"
    ]
    if (pycor mod 5 = 0 or pycor mod 5 = 1) and not (pxcor mod 5 = 0 or pxcor mod 5 = 1) [
      set road-direction "horizontal"
    ]
    if (pxcor mod 5 = 0 or pxcor mod 5 = 1) and not (pycor mod 5 = 0 or pycor mod 5 = 1) [
      set road-direction "vertical"
    ]
  ]
end

;; --- Remove stoplights at the edges ---
to setup-traffic-lights
  let min-x min [pxcor] of patches
  let max-x max [pxcor] of patches
  let min-y min [pycor] of patches
  let max-y max [pycor] of patches

  ;; Only create stoplights at the center of each intersection (not every patch)
  let intersection-centers patches with [
    is-intersection and
    pxcor > min-x + 1 and pxcor < max-x - 1 and
    pycor > min-y + 1 and pycor < max-y - 1 and
    pxcor mod 5 = 0 and pycor mod 5 = 0  ;; Only the center patch of each 2x2 intersection
  ]

  ask intersection-centers [
    ;; initialize per-intersection controller with randomized threshold within [min,max]
    let minC max list 1 floor minimum-traffic-light-cycle-time
    let maxC max list minC floor maximum-traffic-light-cycle-time
    set sig-phase "vertical"
    set sig-timer 0
    set sig-threshold minC + random (maxC - minC + 1)

    ;; top-left -> south (vertical)
    sprout-traffic-lights 1 [
      set state "green"
      set light-timer 0
      set cycle-time maximum-traffic-light-cycle-time
      set direction "south"
      set color green
      set shape "circle"
      set size 1.5
      setxy xcor - 1 ycor + 1
    ]

    ;; top-right -> west (horizontal)
    sprout-traffic-lights 1 [
      set state "red"
      set light-timer maximum-traffic-light-cycle-time
      set cycle-time maximum-traffic-light-cycle-time
      set direction "west"
      set color red
      set shape "circle"
      set size 1.5
      setxy xcor + 1 ycor + 1
    ]

    ;; bottom-left -> east (horizontal)
    sprout-traffic-lights 1 [
      set state "red"
      set light-timer maximum-traffic-light-cycle-time
      set cycle-time maximum-traffic-light-cycle-time
      set direction "east"
      set color red
      set shape "circle"
      set size 1.5
      setxy xcor - 1 ycor - 1
    ]

    ;; bottom-right -> north (vertical)
    sprout-traffic-lights 1 [
      set state "green"
      set light-timer 0
      set cycle-time maximum-traffic-light-cycle-time
      set direction "north"
      set color green
      set shape "circle"
      set size 1.5
      setxy xcor + 1 ycor - 1
    ]
  ]
end

;; --- Only assign valid, reachable destinations to cars ---
to setup-cars
  let min-x min [pxcor] of patches
  let max-x max [pxcor] of patches
  let min-y min [pycor] of patches
  let max-y max [pycor] of patches

  ;; Spawn cars throughout the road network, excluding edge lanes used as destinations
  let road-patches patches with [
    is-road and not is-intersection and
    pxcor > min-x + 2 and pxcor < max-x - 2 and
    pycor > min-y + 2 and pycor < max-y - 2
  ]

  let num-cars floor (count road-patches * initial-car-density / 100)
  ask n-of num-cars road-patches [
    sprout-cars 1 [
      ;; One-way travel: determine road orientation and set heading + destination to edge
      let rd [road-direction] of patch-here
      let left-edge   (min-x + 2)
      let right-edge  (max-x - 2)
      let bottom-edge (min-y + 2)
      let top-edge    (max-y - 2)
      let span-x right-edge - left-edge
      let span-y top-edge - bottom-edge
      ;; Minimum trip thresholds (25% of axis span, but at least 3 patches)
      let min-trip-x max list 3 (span-x * 0.25)
      let min-trip-y max list 3 (span-y * 0.25)

      if rd = "vertical" [
        let dist-top    (top-edge - pycor)
        let dist-bottom (pycor - bottom-edge)
        let nearest-top? dist-top < dist-bottom
        ifelse (min list dist-top dist-bottom) < min-trip-y [
          ifelse nearest-top? [ set heading 180 set destination-x pxcor set destination-y bottom-edge ]
          [ set heading 0 set destination-x pxcor set destination-y top-edge ]
        ] [
          ifelse nearest-top? [ set heading 0 set destination-x pxcor set destination-y top-edge ]
          [ set heading 180 set destination-x pxcor set destination-y bottom-edge ]
        ]
      ]
      if rd = "horizontal" [
        let dist-right (right-edge - pxcor)
        let dist-left  (pxcor - left-edge)
        let nearest-right? dist-right < dist-left
        ifelse (min list dist-right dist-left) < min-trip-x [
          ifelse nearest-right? [ set heading 270 set destination-x left-edge set destination-y pycor ]
          [ set heading 90 set destination-x right-edge set destination-y pycor ]
        ] [
          ifelse nearest-right? [ set heading 90 set destination-x right-edge set destination-y pycor ]
          [ set heading 270 set destination-x left-edge set destination-y pycor ]
        ]
      ]

      ;; Final guard: if already at/very near destination, flip to opposite edge
      if abs (pxcor - destination-x) <= 0.5 and abs (pycor - destination-y) <= 0.5 [
        if heading = 0  [ set heading 180 set destination-y (min-y + 2) ]
        if heading = 180 [ set heading 0   set destination-y (max-y - 2) ]
        if heading = 90  [ set heading 270 set destination-x (min-x + 2) ]
        if heading = 270 [ set heading 90  set destination-x (max-x - 2) ]
      ]

      ;; Standard init
      set travel-time 0
      set route []
      set speed 1
      set max-speed 2
      set waiting-time 0
      set is-waiting false
      set color blue
      set shape "car"
      set size 1
      set lane 0

      ;; Position car in the correct lane based on heading
      position-in-correct-lane
    ]
    set car-here true
  ]
  set total-cars num-cars
end

to-report toward-general-direction
  ;; Determine a general direction based on spawn position
  let min-x min [pxcor] of patches
  let max-x max [pxcor] of patches
  let min-y min [pycor] of patches
  let max-y max [pycor] of patches

  ;; Choose a random direction that makes sense for the spawn position
  let choices []

  ;; Can go north if not at top
  if pycor < max-y - 1 [ set choices lput 0 choices ]
  ;; Can go south if not at bottom
  if pycor > min-y + 1 [ set choices lput 180 choices ]
  ;; Can go east if not at right
  if pxcor < max-x - 1 [ set choices lput 90 choices ]
  ;; Can go west if not at left
  if pxcor > min-x + 1 [ set choices lput 270 choices ]

  ;; Return a random choice from available directions
  ifelse length choices > 0 [
    report one-of choices
  ] [
    ;; Fallback to north if somehow no choices available
    report 0
  ]
end

to position-in-correct-lane
  ;; Align car to the correct lane for right-hand traffic
  ;; Vertical roads use pxcor mod 5 = 0/1 lanes (x changes). Horizontal roads use pycor mod 5 = 0/1 lanes (y changes)
  if heading = 0 [
    ;; Moving north: right lane of vertical road -> pxcor mod 5 = 1
    if (pxcor mod 5) != 1 [ set xcor xcor + 1 ]
  ]
  if heading = 180 [
    ;; Moving south: right lane (relative) -> west lane -> pxcor mod 5 = 0
    if (pxcor mod 5) != 0 [ set xcor xcor - 1 ]
  ]
  if heading = 90 [
    ;; Moving east: right lane of horizontal road -> pycor mod 5 = 0 (bottom)
    if (pycor mod 5) != 0 [ set ycor ycor - 1 ]
  ]
  if heading = 270 [
    ;; Moving west: right lane of horizontal road -> pycor mod 5 = 1 (top)
    if (pycor mod 5) != 1 [ set ycor ycor + 1 ]
  ]
end

to generate-route
  ;; Simplified route generation - just set destination
  ;; The car will navigate using heading adjustments at intersections
  set route []
end

;; ==========================================
;; GO PROCEDURE
;; ==========================================

to go
  if not any? cars [ stop ]

  set time-step time-step + 1

  ;; Update traffic lights
  update-traffic-lights

  ;; Ensure cars maintain their visuals
  ask cars [
    set shape "car"
    set color blue
  ]

  ;; Update car decisions and movements
  ask cars [
    make-decision
  ]

  ;; Move cars simultaneously
  ask cars [
    move-car
  ]

  ;; Update environment
  update-environment

  ;; Collect data
  collect-data

  tick
end

;; ==========================================
;; TRAFFIC LIGHT LOGIC
;; ==========================================

to update-traffic-lights
  ;; Each intersection center patch controls its four lights with its own randomized cycle
  ask patches with [is-intersection and pxcor mod 5 = 0 and pycor mod 5 = 0] [
    set sig-timer sig-timer + 1

    let minC max list 1 floor minimum-traffic-light-cycle-time
    let maxC max list minC floor maximum-traffic-light-cycle-time

    if sig-timer >= sig-threshold [
      ifelse sig-phase = "vertical" [ set sig-phase "horizontal" ] [ set sig-phase "vertical" ]
      set sig-timer 0
      set sig-threshold minC + random (maxC - minC + 1)
    ]

    let v-lights traffic-lights with [
      abs (xcor - [pxcor] of myself) = 1 and abs (ycor - [pycor] of myself) = 1 and
      (direction = "north" or direction = "south")
    ]
    let h-lights traffic-lights with [
      abs (xcor - [pxcor] of myself) = 1 and abs (ycor - [pycor] of myself) = 1 and
      (direction = "east" or direction = "west")
    ]

    ifelse sig-phase = "vertical" [
      ask v-lights [ set state "green" set color green ]
      ask h-lights [ set state "red"   set color red ]
    ] [
      ask v-lights [ set state "red"   set color red ]
      ask h-lights [ set state "green" set color green ]
    ]
  ]
end

;; ==========================================
;; CAR DECISION LOGIC (respect red lights)
;; ==========================================

to make-decision
  ;; Check if car has reached its specific destination edge (with some tolerance)
  if (abs (pxcor - destination-x) <= 1 and abs (pycor - destination-y) <= 1) [
    set cars-completed cars-completed + 1
    set sum-travel-time-completed sum-travel-time-completed + travel-time
    ask patch-here [ set car-here false ]
    die
    stop
  ]

  ;; Edge U-turn logic
  let min-x min [pxcor] of patches
  let max-x max [pxcor] of patches
  let min-y min [pycor] of patches
  let max-y max [pycor] of patches
  if (pxcor = min-x + 2 or pxcor = max-x - 2 or pycor = min-y + 2 or pycor = max-y - 2) [
    if not (abs (pxcor - destination-x) <= 1 and abs (pycor - destination-y) <= 1) [
      do-u-turn
    ]
  ]

  ;; Deterministic red-light compliance using the intersection patch ahead
  let ip patch-ahead 1
  if ip != nobody [
    ;; If the patch ahead is the intersection, we are about to enter it
    if [is-intersection] of ip and not [is-intersection] of patch-here [
      let ctl one-of traffic-lights with [
        abs (xcor - [pxcor] of ip) <= 1 and abs (ycor - [pycor] of ip) <= 1 and
        ((direction = "north" and [heading] of myself = 0) or
         (direction = "south" and [heading] of myself = 180) or
         (direction = "east"  and [heading] of myself = 90) or
         (direction = "west"  and [heading] of myself = 270))
      ]
      if ctl != nobody and [state] of ctl = "red" [
        set is-waiting true
        set waiting-time waiting-time + 1
        stop
      ]
    ]
  ]

  ;; Gridlock escape: if we are already on the intersection and waited long enough, allow move
  if [is-intersection] of patch-here and waiting-time > 10 [
    set is-waiting false
    set waiting-time 0
  ]

  ;; Same-direction car blocking
  let ahead-patch patch-ahead 1
  if ahead-patch != nobody and [car-here] of ahead-patch [
    let car-ahead one-of cars with [xcor = [pxcor] of ahead-patch and ycor = [pycor] of ahead-patch and heading = [heading] of myself]
    if car-ahead != nobody [
      set is-waiting true
      set waiting-time waiting-time + 1
      stop
    ]
  ]

  ;; Go straight through intersections (no turns)
  if [is-intersection] of patch-here [ ]

  ;; Clear waiting status if nothing blocked us in this tick
  set is-waiting false
  set waiting-time 0
end

to do-u-turn
  ;; Turn the car 180 degrees
  if heading = 0 [ set heading 180 ]      ;; North to South
  if heading = 90 [ set heading 270 ]     ;; East to West
  if heading = 180 [ set heading 0 ]      ;; South to North
  if heading = 270 [ set heading 90 ]     ;; West to East

  ;; Adjust to correct lane for new direction
  adjust-to-correct-lane
end

;; Avoid turning: cars go straight through intersections
to handle-intersection
end

to adjust-to-correct-lane
  ;; After a heading change (e.g., U-turn), snap to correct lane for new direction
  if heading = 0 [
    if (pxcor mod 5) != 1 [ set xcor xcor + 1 ]
  ]
  if heading = 180 [
    if (pxcor mod 5) != 0 [ set xcor xcor - 1 ]
  ]
  if heading = 90 [
    if (pycor mod 5) != 0 [ set ycor ycor - 1 ]
  ]
  if heading = 270 [
    if (pycor mod 5) != 1 [ set ycor ycor + 1 ]
  ]
end

to turn-left
  if heading = 0 [ set heading 270 ]
  if heading = 90 [ set heading 0 ]
  if heading = 180 [ set heading 90 ]
  if heading = 270 [ set heading 180 ]
end

to turn-right
  if heading = 0 [ set heading 90 ]
  if heading = 90 [ set heading 180 ]
  if heading = 180 [ set heading 270 ]
  if heading = 270 [ set heading 0 ]
end

;; ==========================================
;; CAR MOVEMENT
;; ==========================================

to move-car
  if is-waiting [ stop ]

  ;; Update travel time
  set travel-time travel-time + 1

  ;; Optimized speed adjustment
  adjust-speed

  ;; Check if the patch ahead is a road and move
  let ahead-patch patch-ahead 1
  if ahead-patch != nobody and [is-road] of ahead-patch [
    ;; Move forward
    let old-patch patch-here
    forward speed

    ;; Ensure heading stays in cardinal directions
    if heading != 0 and heading != 90 and heading != 180 and heading != 270 [
      set heading round (heading / 90) * 90
    ]

    ;; Keep car in the correct lane after movement
    position-in-correct-lane

    ;; Update patch occupancy efficiently
    ask old-patch [ set car-here false ]
    ask patch-here [ set car-here true ]
  ]
end

to adjust-speed
  ;; Optimized speed adjustment with better logic
  let cars-ahead cars in-cone 2 20 with [heading = [heading] of myself]

  ifelse count cars-ahead > 0 [
    ;; Slow down if cars ahead (maintain safe distance)
    let closest-car min-one-of cars-ahead [distance myself]
    let distance-to-car [distance myself] of closest-car

    ifelse distance-to-car < 2 [
      set speed max list 0.2 (speed - 0.3)  ;; Slow down more if very close
    ] [
      set speed max list 0.5 (speed - 0.2)  ;; Moderate slowdown
    ]
  ] [
    ;; Speed up if no obstacles
    ifelse speed < max-car-speed [
      set speed min list max-car-speed (speed + 0.3)  ;; Accelerate faster
    ] [
      set speed max-car-speed  ;; Maintain max speed
    ]
  ]

  ;; Ensure speed stays within bounds
  set speed max list 0.1 (min list max-car-speed speed)
end

;; ==========================================
;; ENVIRONMENT UPDATE
;; ==========================================

to update-environment
  ;; Update congestion level
  set congestion-level count cars with [is-waiting]
end

;; ==========================================
;; DATA COLLECTION
;; ==========================================

to collect-initial-data
  set simulation-data []
end

to collect-data
  ;; Throughput = completed cars per tick (time-step)
  if time-step > 0 [
    set throughput cars-completed / time-step
  ]

  ;; Store data point (time, congestion, throughput)
  set simulation-data lput (list time-step congestion-level throughput) simulation-data
end

to collect-data-now
  ;; Manual data collection procedure for interface button
  collect-data
  print (word "Data collected at time step: " time-step)
  print (word "Congestion level: " congestion-level)
  print (word "Throughput: " throughput)
end

;; ==========================================
;; HELPER PROCEDURES
;; ==========================================

to-report lights-at-patch [target-patch]
  report traffic-lights with [xcor = [pxcor] of target-patch and ycor = [pycor] of target-patch]
end

;; ==========================================
;; MONITORING PROCEDURES
;; ==========================================

to export-data
  ;; Export simulation data to CSV
  csv:to-file "traffic_simulation_data.csv" simulation-data
end

to reset-simulation
  setup
end

;; ==========================================
;; DEBUG PROCEDURES
;; ==========================================

to debug-stuck-cars
  ;; Report information about potentially stuck cars
  let stuck-cars cars with [waiting-time > 10]
  if any? stuck-cars [
    print (word "Found " count stuck-cars " potentially stuck cars:")
    ask stuck-cars [
      print (word "Car at (" xcor ", " ycor ") heading " heading " waiting for " waiting-time " ticks")
      print (word "  Destination: (" destination-x ", " destination-y ")")
      print (word "  Is waiting: " is-waiting)
      print (word "  Travel time: " travel-time)
    ]
  ]

  let non-moving-cars cars with [travel-time > 50 and waiting-time = 0]
  if any? non-moving-cars [
    print (word "Found " count non-moving-cars " cars that haven't moved much:")
    ask non-moving-cars [
      print (word "Car at (" xcor ", " ycor ") heading " heading " travel time: " travel-time)
    ]
  ]
end

to-report is-valid-road-direction [target-heading]
  ;; Check if a heading is valid for the current road
  if [road-direction] of patch-here = "vertical" [
    report target-heading = 0 or target-heading = 180
  ]
  if [road-direction] of patch-here = "horizontal" [
    report target-heading = 90 or target-heading = 270
  ]
  report false
end

to-report get-opposite-lane-position [current-heading]
  ;; Get the lane position for the opposite direction
  if current-heading = 0 [ report 1 ]  ;; North -> right lane
  if current-heading = 180 [ report 0 ] ;; South -> left lane
  if current-heading = 90 [ report 0 ]  ;; East -> bottom lane
  if current-heading = 270 [ report 1 ] ;; West -> top lane
  report 0
end
@#$#@#$#@
GRAPHICS-WINDOW
619
10
1303
695
-1
-1
11.082
1
10
1
1
1
0
0
0
1
-30
30
-30
30
0
0
1
ticks
30.0

BUTTON
60
10
124
43
Setup
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
199
10
262
43
go
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
0
82
172
115
initial-car-density
initial-car-density
5
15
15.0
5
1
NIL
HORIZONTAL

SLIDER
0
176
219
209
maximum-traffic-light-cycle-time
maximum-traffic-light-cycle-time
5
50
25.0
5
1
NIL
HORIZONTAL

SLIDER
0
228
172
261
max-car-speed
max-car-speed
1
10
5.0
1
1
NIL
HORIZONTAL

MONITOR
333
76
400
121
Time Step
time-step
17
1
11

MONITOR
427
78
497
123
Total Cars
total-cars
17
1
11

MONITOR
335
158
436
203
Cars Completed
cars-completed
17
1
11

MONITOR
469
158
596
203
Throughput
throughput
17
1
11

MONITOR
335
232
443
277
Congestion Level
congestion-level
17
1
11

PLOT
0
497
200
647
Congestion over Time
NIL
NIL
0.0
10.0
0.0
10.0
true
false
"" ""
PENS
"default" 1.0 0 -16777216 true "" "plot congestion-level"

PLOT
202
497
402
647
Throughput over Time
NIL
NIL
0.0
10.0
0.0
10.0
true
false
"" ""
PENS
"default" 1.0 0 -16777216 true "" "plot throughput * 100"

SLIDER
0
136
221
169
minimum-traffic-light-cycle-time
minimum-traffic-light-cycle-time
5
25
25.0
5
1
NIL
HORIZONTAL

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
NetLogo 6.4.0
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
