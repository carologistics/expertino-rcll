; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.

(defrule refbox-recv-GameState
  ?pb-msg <- (protobuf-msg (type "llsf_msgs.GameState") (ptr ?p))
  ?gt <- (game-time ?old-game-time)
  ?gs <- (game-state
    (points ?points)
    (points-other ?points-other)
    (team ?team)
    (team-color ?team-color)
    (team-other ?team-other)
  )
  =>
  (bind ?new-team-color ?team-color)
  (bind ?new-team-other ?team-other)
  (bind ?new-points ?points)
  (bind ?new-points-other ?points-other)
  (retract ?gt)
  (bind ?time (pb-field-value ?p "game_time"))
  (bind ?sec (pb-field-value ?time "sec"))
  (bind ?nsec (pb-field-value ?time "nsec"))
  (assert (game-time (+ ?sec (/  ?nsec 1000000))))
  (if (and (pb-has-field ?p "team_cyan")
           (eq (pb-field-value ?p "team_cyan") ?team))
    then 
      (bind ?new-team-color CYAN)
      (bind ?new-points (pb-field-value ?p "points_cyan"))
      (if (pb-has-field ?p "team_magenta") then
        (bind ?new-team-other (pb-field-value ?p "team_magenta"))
        (bind ?new-points-other (pb-field-value ?p "points_magenta"))
      )
  )
  (if (and (pb-has-field ?p "team_magenta")
           (eq (pb-field-value ?p "team_magenta") ?team))
    then 
      (bind ?new-team-color MAGENTA)
      (bind ?new-points (pb-field-value ?p "points_magenta"))
      (if (pb-has-field ?p "team_cyan") then
        (bind ?new-team-other (pb-field-value ?p "team_cyan"))
        (bind ?new-points-other (pb-field-value ?p "points_cyan"))
      )
  )
  (if (and (neq ?new-team-color ?team-color) (neq ?team-color NOT-SET)) then
    (printout warn "Switching team color from " ?team-color " to " ?new-team-color crlf)
  )
  (bind ?new-info (assert (game-state
    (team ?team)
    (team-other ?new-team-other)
    (team-color ?new-team-color)
    (points ?new-points)
    (points-other ?new-points-other)
    (state (pb-field-value ?p "state"))
    (phase (pb-field-value ?p "phase"))
    (field-height (pb-field-value ?p "field_height"))
    (field-width (pb-field-value ?p "field_width"))
    (field-mirrored (pb-field-value ?p "field_mirrored"))
  )))
  (if ?new-info then (retract ?gs))
  (retract ?pb-msg)
)

(defrule refbox-recv-MachineInfo
  ?pb-msg <- (protobuf-msg (type "llsf_msgs.MachineInfo") (ptr ?p))
  (game-state (team-color ?team-color))
  =>
  (foreach ?m (pb-field-list ?p "machines")
    (bind ?m-name (sym-cat (pb-field-value ?m "name")))
    (bind ?m-type (sym-cat (pb-field-value ?m "type")))
    (bind ?m-team (sym-cat (pb-field-value ?m "team_color")))
    (bind ?m-state (sym-cat (pb-field-value ?m "state")))
    ; ground truth info
    (bind ?rot  FALSE)
    (bind ?zone NOT-SET)
    (if (pb-has-field ?m "rotation") then
      (bind ?rot  (pb-field-value ?m "rotation"))
    )
    (if (pb-has-field ?m "zone") then
      (bind ?zone (pb-field-value ?m "zone"))
    )
    (if (eq ?m-type RS) then
      (assert (ring-assignment (machine ?m-name) (colors (pb-field-list ?m "ring_colors"))))
    )
    (assert (machine (name ?m-name) (type ?m-type) (team-color ?m-team) (state ?m-state) (zone ?zone) (rotation ?rot)))
  )
  (delayed-do-for-all-facts ((?m1 machine) (?m2 machine)) (and (< (fact-index ?m1) (fact-index ?m2)) (eq ?m1:name ?m2:name))
    (retract ?m1)
  )
  (delayed-do-for-all-facts ((?ra1 ring-assignment) (?ra2 ring-assignment)) (and (< (fact-index ?ra1) (fact-index ?ra2)) (eq ?ra1:machine ?ra2:machine))
   (retract ?ra1)
  )
  (retract ?pb-msg)
)

(defrule refbox-recv-RingInfo
  ?pb-msg <- (protobuf-msg (type "llsf_msgs.RingInfo") (ptr ?p))
  =>
  (foreach ?r (pb-field-list ?p "rings")
    (bind ?color (pb-field-value ?r "ring_color"))
    (bind ?raw-material (pb-field-value ?r "raw_material"))
    (assert (ring-spec (color ?color) (cost ?raw-material)))
  )
  (delayed-do-for-all-facts ((?rs1 ring-spec) (?rs2 ring-spec)) (and (< (fact-index ?rs1) (fact-index ?rs2)) (eq ?rs1:color ?rs2:color))
   (retract ?rs1)
  )
)

(defrule refbox-recv-RobotInfo
  "Receive robot state information to detect if a robot is placed into (or
	recovered from) maintenance."
  ?pb-msg <- (protobuf-msg (type "llsf_msgs.RobotInfo") (ptr ?r))
  (game-state (team ?team) (team-color ?team-color))
  =>
  (foreach ?p (pb-field-list ?r "robots")
    (if (and (eq ?team (pb-field-value ?p "team"))
      (eq ?team-color (pb-field-value ?p "team_color"))) then
      (bind ?state (sym-cat (pb-field-value ?p "state")))
      (bind ?name (sym-cat (pb-field-value ?p "name")))
      (bind ?old-state nil)
      (do-for-fact ((?robot robot)) (eq ?robot:name ?name)
        (bind ?old-state ?robot:state)
        (if (and (eq ?old-state MAINTENANCE)
               (eq ?state ACTIVE))
         then
          (modify ?robot (state ?state) (is-busy FALSE))
        )
        (if (and (eq ?old-state ACTIVE)
                 (neq ?state ACTIVE))
         then
          (modify ?robot (state ?state) (is-busy TRUE))
        )
      )
    )
  )
  (retract ?pb-msg)
)

(defrule refbox-recv-OrderInfo
  "Assert products sent by the refbox."
  ?pb-msg <- (protobuf-msg (type "llsf_msgs.OrderInfo") (ptr ?ptr))
  (game-state (team ?team) (team-color ?team-color))
  =>
  (foreach ?o (pb-field-list ?ptr "orders")
    (bind ?id (pb-field-value ?o "id"))
    (bind ?name (sym-cat O ?id))
    ;check if the order is new
    (bind ?complexity (pb-field-value ?o "complexity"))
    (bind ?competitive (pb-field-value ?o "competitive"))
    (bind ?quantity-requested (pb-field-value ?o "quantity_requested"))
    (bind ?begin (pb-field-value ?o "delivery_period_begin"))
    (bind ?end (pb-field-value ?o "delivery_period_end"))
    (if (pb-has-field ?o "base_color") then
      (bind ?base (pb-field-value ?o "base_color"))
    else
      (bind ?base UNKNOWN)
    )
    (bind ?cap (pb-field-value ?o "cap_color"))
    (bind ?ring-colors (pb-field-list ?o "ring_colors"))
    (if (eq ?team-color CYAN) then
      (bind ?qd-them (pb-field-value ?o "quantity_delivered_magenta"))
      (bind ?qd-us (pb-field-value ?o "quantity_delivered_cyan"))
    else
      (bind ?qd-them (pb-field-value ?o "quantity_delivered_cyan"))
      (bind ?qd-us (pb-field-value ?o "quantity_delivered_magenta"))
    )
    (assert (order 
      (id ?id)
      (name ?name)
      (complexity ?complexity)
      (competitive ?competitive)
      (quantity-requested ?quantity-requested)
      (delivery-begin ?begin)
      (delivery-end ?end)
      (base-color ?base)
      (ring-colors ?ring-colors)
      (cap-color ?cap)
      (quantity-delivered ?qd-us)
      (quantity-delivered-other ?qd-them)
    ))
  )
  (delayed-do-for-all-facts ((?o1 order) (?o2 order)) (and (< (fact-index ?o1) (fact-index ?o2)) (eq ?o1:id ?o2:id))
   (retract ?o1)
  )
  (retract ?pb-msg)
)
