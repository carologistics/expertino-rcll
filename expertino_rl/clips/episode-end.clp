(defrule episode-end
  (declare (salience 1))
  (game-state (phase POST_GAME))
  (not (rl-episode-end (success TRUE)))
  =>
  (assert (rl-episode-end (success TRUE)))
)

(defrule episode-end-finish-rl-action
  (declare (salience 1))
  (rl-episode-end (success TRUE))
  ?a <- (rl-action (is-selected TRUE) (is-finished FALSE))
  =>
  (modify ?a (reward ?*POINTS-EPISODE-END-SUCCESS*) (is-finished TRUE))
)

(defrule rl-stop-agent-on-training-end
  (rl-end-training)
=>
  (cx-shutdown)
)

(defrule refbox-failure-episode-end
  (not (rl-episode-end (success ?success)))
  (last-game-time (last-time ?time))
  (time ?now)
  (test (< ?time (- ?now 20)))
  =>
  (printout error "Refbox failure, restarting... " crlf)
  (assert (rl-episode-end (success TRUE)))
)