(defrule refbox-action-mps-prepare-send-signal
  ;(time $?now)
  (game-state (team-color ?team-color) (phase PRODUCTION))
  ?ex <- (executor (worker REFBOX) (state INIT) (pddl-action-id ?action-id)) 
  (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  (protobuf-peer (name refbox-private) (peer-id ?peer-id))
  => 
  (bind ?machine-instruction (pb-create "llsf_msgs.PrepareMachine"))
  (pb-set-field ?machine-instruction "team_color" ?team-color)
  (switch ?action-name
    (case bs-dispense
      then
        (bind ?mps (pddl-place-to-refbox-mps (nth$ 2 ?action-params) ?team-color))
        (bind ?bs-inst (pb-create "llsf_msgs.PrepareInstructionBS"))
        (pb-set-field ?bs-inst "side" (pddl-place-to-mps-side (nth$ 3 ?action-params)))
        (pb-set-field ?bs-inst "color" (pddl-task-to-base-color (nth$ 4 ?action-params)))
        (pb-set-field ?machine-instruction "instruction_bs" ?bs-inst)
    )
    (case dispense-pay
      then
        (bind ?mps (pddl-place-to-refbox-mps (nth$ 3 ?action-params) ?team-color))
        (bind ?color (nth$ (random 1 3) (create$ BASE_RED BASE_BLACK BASE_SILVER)))
        (bind ?bs-inst (pb-create "llsf_msgs.PrepareInstructionBS"))
        (pb-set-field ?bs-inst "side" (pddl-place-to-mps-side (nth$ 4 ?action-params)))
        (pb-set-field ?bs-inst "color" ?color)
        (pb-set-field ?machine-instruction "instruction_bs" ?bs-inst)
    )
    (case cs-mount-cap
      then
        (bind ?mps (pddl-place-to-refbox-mps (nth$ 2 ?action-params) ?team-color))
        ;(bind ?color (pddl-task-to-cap-color (nth$ 5 ?action-params)))
      	(bind ?cs-inst (pb-create "llsf_msgs.PrepareInstructionCS"))
        (pb-set-field ?cs-inst "operation"  MOUNT_CAP)
        (pb-set-field ?machine-instruction "instruction_cs" ?cs-inst)
    )
    (case cs-buffer
      then
        (bind ?mps (pddl-place-to-refbox-mps (nth$ 2 ?action-params) ?team-color))
      	(bind ?cs-inst (pb-create "llsf_msgs.PrepareInstructionCS"))
        (pb-set-field ?cs-inst "operation" RETRIEVE_CAP)
        (pb-set-field ?machine-instruction "instruction_cs" ?cs-inst)
    )
    (case rs-mount-ring
      then
        (bind ?mps (pddl-place-to-refbox-mps (nth$ 2 ?action-params) ?team-color))
        (bind ?rs-inst (pb-create "llsf_msgs.PrepareInstructionRS"))
        (pb-set-field ?rs-inst "ring_color" (pddl-task-to-ring-color (nth$ 5 ?action-params)))
        (pb-set-field ?machine-instruction "instruction_rs" ?rs-inst)
    )
    (case finalize
      then
        (bind ?ds-inst (pb-create "llsf_msgs.PrepareInstructionDS"))
;        (bind ?order (nth$ 1 ?instruction_info))
;        (bind ?order-id (float (string-to-field (sub-string 2 (length$ (str-cat ?order)) (str-cat ?order)))))
;        (pb-set-field ?ds-inst "order_id" ?order-id)
        (pb-set-field ?machine-instruction "instruction_ds" ?ds-inst)
    )
  )
  (pb-set-field ?machine-instruction "machine" (str-cat ?mps))
  (pb-broadcast ?peer-id ?machine-instruction)
  (pb-destroy ?machine-instruction)
  (printout t "Sent Prepare Msg for " ?mps " with " ?action-params crlf)
  (modify ?ex (state ACCEPTED))
)

(defrule executor-mps-prepare-done
  (machine (name ?mps) (state READY-AT-OUTPUT|PREPARED)) 
  ?ex <- (executor (pddl-action-id ?action-id) (worker REFBOX) (state ACCEPTED))
  ?pa <- (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  (game-state (team-color ?team-color) (phase PRODUCTION))
  =>
  (if (eq ?action-name dispense-pay)
   then
    (bind ?o-mps (pddl-place-to-refbox-mps (nth$ 3 ?action-params) ?team-color))
   else
    (bind ?o-mps (pddl-place-to-refbox-mps (nth$ 2 ?action-params) ?team-color))
  )
  (if (eq ?o-mps ?mps)
   then
    (modify ?ex (state SUCCEEDED))
    (assert (pddl-action-apply-effect (action ?action-id) (effect-type END)))
  ) 
)
