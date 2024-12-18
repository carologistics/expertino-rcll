(defrule refbox-action-populate-executor-params
  ?ex <- (executor (id ?ex-id) (worker REFBOX) (state ASSIGNED)
               (pddl-action-id ?action-id) (param-names) (param-values))
  ?pa <- (pddl-action (id ?action-id) (name ?name) (params $?action-params))
  (game-state (team-color ?team-color)) 
  =>
  (bind ?param-names FALSE)
  (switch ?name
    (case bs-dispense
      then
        (bind ?wp (first$ ?action-params))
        (bind ?mps (pddl-place-to-refbox-mps (nth$ 2 ?action-params) ?team-color))
        (bind ?mps-side (pddl-place-to-mps-side (nth$ 3 ?action-params)))
        (bind ?color (pddl-task-to-base-color (nth$ 4 ?action-params)))
        (bind ?param-names (create$ wp mps mps-side color))
        (bind ?param-values (create$ ?wp ?mps ?mps-side ?color))
    )
    (case dispense-pay
      then
        (bind ?wp (first$ ?action-params))
        (bind ?mps (pddl-place-to-refbox-mps (nth$ 3 ?action-params) ?team-color))
        (bind ?mps-side (pddl-place-to-mps-side (nth$ 4 ?action-params)))
        (bind ?color (nth$ (random 1 3) (create$ BASE_RED BASE_BLACK BASE_SILVER)))
        (bind ?param-names (create$ wp mps mps-side color))
        (bind ?param-values (create$ ?wp ?mps ?mps-side ?color))
    )
    (case cs-mount-cap
      then
        (bind ?wp (first$ ?action-params))
        (bind ?mps (pddl-place-to-refbox-mps (nth$ 2 ?action-params) ?team-color))
        (bind ?color (pddl-task-to-cap-color (nth$ 5 ?action-params)))
        (bind ?param-names (create$ wp mps color))
        (bind ?param-values (create$ ?wp ?mps ?color))
    )
    (case cs-buffer
      then
        (bind ?wp (first$ ?action-params))
        (bind ?mps (pddl-place-to-refbox-mps (nth$ 2 ?action-params) ?team-color))
        (bind ?param-names (create$ wp mps))
        (bind ?param-values (create$ ?wp ?mps))
    )
    (case rs-mount-ring
      then
        (bind ?wp (first$ ?action-params))
        (bind ?mps (pddl-place-to-refbox-mps (nth$ 2 ?action-params) ?team-color))
        (bind ?color (pddl-task-to-ring-color (nth$ 5 ?action-params)))
        (bind ?param-names (create$ wp mps color))
        (bind ?param-values (create$ ?wp ?mps ?color))
    )
;    (case rs-pay
;      then
;        (bind ?wp (first$ ?action-params))
;        (bind ?color (pddl-task-to-cap-color (nth$ 2 ?action-params)))
;        (bind ?payment (nth$ 3 ?action-params))
;        (bind ?mps (pddl-place-to-refbox-mps (nth$ 4 ?action-params) ?team-color))
;        (bind ?mps-side (pddl-place-to-mps-side (nth$ 5 ?action-params)))
;        (bind ?param-names (create$ wp mps mps-side payment color))
;        (bind ?param-values (create$ ?wp ?mps ?mps-side ?payment ?color))
;    )
;    (case rs-pay-with-cc
;      then
;        (bind ?payment (nth$ 1 ?action-params))
;        (bind ?mps (pddl-place-to-refbox-mps (nth$ 2 ?action-params) ?team-color))
;        (bind ?mps-side (pddl-place-to-mps-side (nth$ 3 ?action-params)))
;        (bind ?param-names (create$ mps mps-side payment))
;        (bind ?param-values (create$ ?mps ?mps-side ?payment))
;    )
  )
  (if ?param-names
   then
     (modify ?ex (param-names ?param-names) (param-values ?param-values))
  )
)

(defrule refbox-action-mps-prepare-send-signal
  ;(time $?now)
  ;(not (timer (name ?t-name&:(eq ?t-name (sym-cat ?goal-id - ?plan-id - ?id -send-again-timer)))))
  (game-state (team-color ?team-color) (phase PRODUCTION))
  ?ex <- (executor (worker REFBOX) (state ASSIGNED) (pddl-action-id ?action-id) 
                   (param-names $?param-names&:(neq (length$ $?param-names) 0)) 
                   (param-values $?param-values&:(neq (length$ $?param-values) 0)))
  (pddl-action (id ?action-id) (name ?action-name))
  (protobuf-peer (name refbox-private) (peer-id ?peer-id))
  =>
  (bind ?mps (get-param-by-name mps ?param-names ?param-values)) 
  (bind ?machine-instruction (pb-create "llsf_msgs.PrepareMachine"))
  (pb-set-field ?machine-instruction "team_color" ?team-color)
  (pb-set-field ?machine-instruction "machine" (str-cat ?mps))

  (switch (refbox-mps-to-type ?mps) 
    (case BS 
      then
        (bind ?bs-inst (pb-create "llsf_msgs.PrepareInstructionBS"))
        (pb-set-field ?bs-inst "side" (get-param-by-name mps-side ?param-names ?param-values))
        (pb-set-field ?bs-inst "color" (get-param-by-name color ?param-names ?param-values))
        (pb-set-field ?machine-instruction "instruction_bs" ?bs-inst)
    )
    (case CS
      then
      	(bind ?cs-inst (pb-create "llsf_msgs.PrepareInstructionCS"))
        (switch ?action-name 
          (case cs-mount-cap
            then (pb-set-field ?cs-inst "operation" MOUNT_CAP)
          )
          (case cs-buffer
            then (pb-set-field ?cs-inst "operation" RETRIEVE_CAP)
          )
        ) 
        (pb-set-field ?machine-instruction "instruction_cs" ?cs-inst)
    )
;    (case DS
;      then
;        (bind ?ds-inst (pb-create "llsf_msgs.PrepareInstructionDS"))
;        (bind ?order (nth$ 1 ?instruction_info))
;        (bind ?order-id (float (string-to-field (sub-string 2 (length$ (str-cat ?order)) (str-cat ?order)))))
;        (pb-set-field ?ds-inst "order_id" ?order-id)
;        (pb-set-field ?machine-instruction "instruction_ds" ?ds-inst)
;    )
;    (case SS
;      then
;       (bind ?ss-inst (pb-create "llsf_msgs.PrepareInstructionSS"))
;                (bind ?instruction (nth$ 2 ?instruction_info))
;                (pb-set-field ?ss-inst "operation" ?instruction)
;                (pb-set-field ?machine-instruction "instruction_ss" ?ss-inst)
;    )
    (case RS
      then
        (bind ?rs-inst (pb-create "llsf_msgs.PrepareInstructionRS"))
        (pb-set-field ?rs-inst "ring_color" (get-param-by-name color ?param-names ?param-values))
        (pb-set-field ?machine-instruction "instruction_rs" ?rs-inst)
    )
  )

  (pb-broadcast ?peer-id ?machine-instruction)
  (pb-destroy ?machine-instruction)

;  (assert (timer (name (sym-cat ?goal-id - ?plan-id - ?id -send-again-timer))
;                 (time ?now) (seq 1)))
  (printout t "Sent Prepare Msg for " ?mps " with " ?param-values crlf)
  (modify ?ex (state ACCEPTED))
)

(defrule executor-mps-prepare-done
  ?ex <- (executor (pddl-action-id ?action-id) (worker REFBOX) (state ACCEPTED) (param-names $?param-names) (param-values $?param-values))
  ?pa <- (pddl-action (id ?action-id) (name bs-dispense))
  (machine (name ?mps&:(eq ?mps (get-param-by-name mps ?param-names ?param-values))) (state READY-AT-OUTPUT)) 
  =>
  (modify ?ex (state SUCCEEDED))
)
