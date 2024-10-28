; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.

(deffunction zone-to-proto-format (?zone)
  "Replaces the 2nd character of a zone name with '_'
  @param ?zone id of the zone (eg M-Z22)

  @return the input string with 2nd character replaced with '_'
  "
  (return
    (str-cat (sub-string 1 1 ?zone) "_" (sub-string 3 (str-length ?zone) ?zone))
  )
)

(deffunction deg-to-rad (?deg)
  "Converts an angle in degree to radiant
  @param ?deg angle in degree

  @return angle in radiant
  "
  (bind ?bigrad (* (/ ?deg 360) ?*2PI*))
  (if (> ?bigrad ?*PI*) then
    (return (* -1 (- ?*2PI* ?bigrad)))
  else
    (return ?bigrad)
  )
)

(deffunction plan-assert-action (?name $?param-values)
" Assert an action with a unique id."
	(bind ?id-sym (gensym*))
	(bind ?id-str (sub-string 4 (str-length ?id-sym) (str-cat ?id-sym)))
	(assert (plan-action (id (string-to-field ?id-str)) (action-name ?name) (param-values $?param-values) (robot "ROBOT1")))
)

(deffunction plan-assert-sequential (?plan-name ?goal-id ?robot $?action-tuples)
	(bind ?plan-id (sym-cat ?plan-name (gensym*)))
	(assert (plan (id ?plan-id) (goal-id ?goal-id)))
	(bind ?actions (create$))
	; action tuples might contain FALSE in some cases, filter them out
	(foreach ?pa $?action-tuples
		(if ?pa then
			(bind ?actions (append$ ?actions ?pa))
		)
	)
	(foreach ?pa $?actions
		(modify ?pa (id ?pa-index) (plan-id ?plan-id) (goal-id ?goal-id))
	)
)


(deffunction order-to-int (?order)
  (printout yellow (sub-string 2 (str-length ?order) ?order) crlf)
  (printout yellow (string-to-field (sub-string 2 (str-length ?order) ?order)) crlf)
  (return (integer (string-to-field (sub-string 2 (str-length ?order) ?order))))
)


(deffunction create-task-msg (?agent-task-f ?team-color)
  (bind ?task-seq (fact-slot-value ?agent-task-f task-id))
  (bind ?robot (fact-slot-value ?agent-task-f robot))
  (bind ?task-type (fact-slot-value ?agent-task-f task-type))
  (bind ?m (fact-slot-value ?agent-task-f machine))
  (bind ?side (fact-slot-value ?agent-task-f side))
  (bind ?waypoint (fact-slot-value ?agent-task-f waypoint))
  (bind ?wp (fact-slot-value ?agent-task-f workpiece))
  (bind ?colors (fact-slot-value ?agent-task-f workpiece-colors))
  (bind ?order-id (fact-slot-value ?agent-task-f order))
  (bind ?outcome (fact-slot-value ?agent-task-f outcome))
  (bind ?task-msg (pb-create "llsf_msgs.AgentTask"))
  (bind ?name-length (str-length (str-cat ?robot)))
  (bind ?robot-number (string-to-field (sub-string ?name-length ?name-length (str-cat ?robot))))
  (bind ?task-msg-valid TRUE)
  (pb-set-field ?task-msg "team_color" ?team-color)
  (pb-set-field ?task-msg "task_id" ?task-seq)
  (pb-set-field ?task-msg "robot_id" ?robot-number)
  (bind ?task-info (pb-create (str-cat "llsf_msgs." ?task-type)))
  (bind ?task-field-name (str-cat (lowcase ?task-type)))
  (bind ?param1 "waypoint")
  (bind ?param2 "machine_point")
  (if (member$ ?task-type (create$ BufferStation Retrieve Deliver))
   then
    (bind ?param1 "machine_id")
  )
  (if (eq ?task-type BufferStation)
   then
    (bind ?task-field-name "buffer")
  )
  (if (eq ?task-type ExploreWaypoint)
   then
    (bind ?task-field-name "explore_machine")
  )
  (if (neq ?m UNSET)
   then
    (pb-set-field ?task-info ?param1 ?m)
   else
    (if (neq ?waypoint UNSET)
     then
      (pb-set-field ?task-info ?param1 ?waypoint)
     else
      (bind ?task-msg-valid FALSE)
      (printout error "AgentTask with UNSET param " ?param1 ", skip" crlf)
    )
  )
  (if (neq ?side UNSET) then
    (pb-set-field ?task-info ?param2 ?side)
   else
    (if (eq ?task-type Retrieve) then
      (bind ?task-msg-valid FALSE)
      (printout error "AgentTask Retrive with UNSET param " ?param2 ", skip" crlf)
    )
  )
  (if (and (neq ?wp nil) (eq (length$ ?colors) 5))
   then
    (bind ?wp-description-msg (pb-create "llsf_msgs.WorkpieceDescription"))
    (pb-set-field ?wp-description-msg "base_color" (nth$ 1 ?colors))
    (progn$ (?col (subseq$ ?colors 2 4))
      (pb-add-list ?wp-description-msg "ring_colors" ?col)
    )
    (if (neq (nth$ 5 ?colors) CAP_NONE) then
     (pb-set-field ?wp-description-msg "cap_color" (nth$ 5 ?colors))
    )
    (pb-set-field ?task-msg "workpiece_description" ?wp-description-msg)
  )
  (if (neq ?order-id UNSET) then
    (pb-set-field ?task-msg "order_id" (order-to-int ?order-id))
  )
  (if ?task-msg-valid
   then
    (pb-set-field ?task-msg ?task-field-name ?task-info)
    (return ?task-msg)
   else
    (pb-destroy ?task-info)
    (pb-destroy ?task-msg)
  )
)
