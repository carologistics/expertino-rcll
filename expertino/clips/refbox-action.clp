(defrule refbox-action-mps-prepare-send-signal
  ;(time $?now)
  (game-state (team-color ?team-color) (phase PRODUCTION))
  ?ex <- (executor (worker REFBOX) (state INIT) (pddl-action-id ?action-id)) 
  (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  (protobuf-peer (name refbox-private) (peer-id ?peer-id))
  (machine (name ?mps) (state IDLE))
  (test (or 
    (and (eq ?action-name bs-dispense)
         (eq ?mps (pddl-place-to-refbox-mps (nth$ 5 ?action-params) ?team-color))
    )
    (eq ?mps (pddl-place-to-refbox-mps (nth$ 3 ?action-params) ?team-color))
  ) )
  => 
  (bind ?machine-instruction (pb-create "llsf_msgs.PrepareMachine"))
  (pb-set-field ?machine-instruction "team_color" ?team-color)
  (switch ?action-name
    (case bs-dispense
      then
        (bind ?bs-inst (pb-create "llsf_msgs.PrepareInstructionBS"))
        (pb-set-field ?bs-inst "side" (pddl-place-to-mps-side (nth$ 5 ?action-params)))
        (pb-set-field ?bs-inst "color" (pddl-task-to-base-color (nth$ 2 ?action-params)))
        (pb-set-field ?machine-instruction "instruction_bs" ?bs-inst)
    )
    (case bs-dispense-pay
      then
        (bind ?color (nth$ (random 1 3) (create$ BASE_RED BASE_BLACK BASE_SILVER)))
        (bind ?bs-inst (pb-create "llsf_msgs.PrepareInstructionBS"))
        (pb-set-field ?bs-inst "side" (pddl-place-to-mps-side (nth$ 3 ?action-params)))
        (pb-set-field ?bs-inst "color" ?color)
        (pb-set-field ?machine-instruction "instruction_bs" ?bs-inst)
    )
    (case cs-mount-cap
      then
        ;(bind ?color (pddl-task-to-cap-color (nth$ 5 ?action-params)))
      	(bind ?cs-inst (pb-create "llsf_msgs.PrepareInstructionCS"))
        (pb-set-field ?cs-inst "operation"  MOUNT_CAP)
        (pb-set-field ?machine-instruction "instruction_cs" ?cs-inst)
    )
    (case cs-buffer
      then
      	(bind ?cs-inst (pb-create "llsf_msgs.PrepareInstructionCS"))
        (pb-set-field ?cs-inst "operation" RETRIEVE_CAP)
        (pb-set-field ?machine-instruction "instruction_cs" ?cs-inst)
    )
    (case rs-mount-ring
      then
        (bind ?rs-inst (pb-create "llsf_msgs.PrepareInstructionRS"))
        (pb-set-field ?rs-inst "ring_color" (pddl-task-to-ring-color (nth$ 5 ?action-params)))
        (pb-set-field ?machine-instruction "instruction_rs" ?rs-inst)
    )
    (case finalize
      then
        (bind ?wp (nth$ 1 ?action-params))
        (do-for-fact ((?w-f workpiece-for-order)) (eq ?w-f:wp ?wp)
          (bind ?order-id ?w-f:order)
        )
        (bind ?ds-inst (pb-create "llsf_msgs.PrepareInstructionDS"))
        (pb-set-field ?ds-inst "order_id" (order-to-int ?order-id))
        (pb-set-field ?machine-instruction "instruction_ds" ?ds-inst)
    )
  )
  (pb-set-field ?machine-instruction "machine" (str-cat ?mps))
  (pb-broadcast ?peer-id ?machine-instruction)
  (pb-destroy ?machine-instruction)
  (printout t "Sent Prepare Msg for " ?mps " with " ?action-params crlf)
  (modify ?ex (state ACCEPTED))
)

(defrule executor-mps-ready-at-output
  (machine (name ?mps) (state READY-AT-OUTPUT)) 
  ?ex <- (executor (pddl-action-id ?action-id) (worker REFBOX) (state ACCEPTED))
  ?pa <- (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  (game-state (team-color ?team-color) (phase PRODUCTION))
  =>
  (if (eq ?action-name bs-dispense)
   then
    (bind ?o-mps (pddl-place-to-refbox-mps (nth$ 5 ?action-params) ?team-color))
   else
    (bind ?o-mps (pddl-place-to-refbox-mps (nth$ 3 ?action-params) ?team-color))
  )
  (if (eq ?o-mps ?mps)
   then
    (modify ?ex (state SUCCEEDED))
    (assert (pddl-action-get-effect (apply TRUE) (action ?action-id) (effect-type END)))
  ) 
)

(defrule executor-finalize-mps-processed
  (machine (name ?mps) (state PROCESSED))
  (game-state (team-color ?team-color) (phase PRODUCTION))
  ?ex <- (executor (pddl-action-id ?action-id) (worker REFBOX) (state ACCEPTED))
  ?pa <- (pddl-action (id ?action-id) (name finalize) (params $?action-params))
  (test 
    (eq (pddl-place-to-refbox-mps (nth$ 2 ?action-params) ?team-color) ?mps)
  )
  =>
  (modify ?ex (state SUCCEEDED))
  (assert (pddl-action-get-effect (apply TRUE) (action ?action-id) (effect-type END)))
)
  
(defrule executor-machine-broken
  (machine (name ?mps) (state BROKEN))
  (game-state (team-color ?team-color) (phase PRODUCTION))
  ?ex <- (executor (pddl-action-id ?action-id) (worker REFBOX) (state ACCEPTED))
  ?pa <- (pddl-action (id ?action-id) (name finalize) (params $?action-params))
  (test 
    (or 
      (eq (pddl-place-to-refbox-mps (nth$ 2 ?action-params) ?team-color) ?mps)
      (eq (pddl-place-to-refbox-mps (nth$ 3 ?action-params) ?team-color) ?mps)
      (eq (pddl-place-to-refbox-mps (nth$ 5 ?action-params) ?team-color) ?mps)
    )
  )
  =>
  (modify ?ex (state ABORTED)))
