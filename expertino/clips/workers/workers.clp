(defrule worker-idle-init 
  (worker (id ?worker) (state IDLE))
  (not (worker-idle-timer (worker ?worker)))
  (game-state (phase PRODUCTION))
  (game-time ?time)
  =>
  (assert (worker-idle-timer (worker ?worker) (start-time ?time)))
)

(defrule worker-idle-reset
  (worker (id ?worker) (state ~IDLE))
  ?timer <- (worker-idle-timer (worker ?worker))
  =>
  (retract ?timer)
)

(defrule worker-set-busy
  ?worker <- (worker (id ?worker-id) (state IDLE))
  (agenda-action-item (execution-state SELECTED|EXECUTING) (worker ?worker-id))
  =>
  (modify ?worker (state BUSY))
)

(defrule worker-set-idle
  ?worker <- (worker (id ?worker-id) (state BUSY))
  ?aa <- (agenda-action-item (execution-state COMPLETED) (worker ?worker-id))
  =>
  (modify ?worker (state IDLE))
  (modify ?aa (worker UNSET))
)

(defrule worker-set-recovery
  ?worker <- (worker (id ?worker-id) (state BUSY|IDLE))
  ?aa <- (agenda-action-item (execution-state ERROR) (worker ?worker-id))
  =>
  (modify ?worker (state RECOVERY))
  (modify ?aa (worker UNSET))
)

