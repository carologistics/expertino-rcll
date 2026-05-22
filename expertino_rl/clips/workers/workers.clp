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
  (executor (state INIT|ACCEPTED|REQUESTED) (worker ?worker-id))
  =>
  (modify ?worker (state BUSY))
)

(defrule worker-set-idle
  ?worker <- (worker (id ?worker-id) (state BUSY))
  ?ex <- (executor (state SUCCEEDED) (worker ?worker-id))
  =>
  (modify ?worker (state IDLE))
  (modify ?ex (worker UNSET))
)

(defrule worker-set-recovery
  ?worker <- (worker (id ?worker-id) (state BUSY|IDLE))
  ?ex <- (executor (state CANCELLED|ABORTED) (worker ?worker-id))
  =>
  (modify ?worker (state RECOVERY))
  (modify ?ex (worker UNSET))
)

