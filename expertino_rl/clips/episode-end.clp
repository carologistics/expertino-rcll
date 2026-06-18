(defrule episode-end
  (declare (salience 1))
  (game-state (phase POST_GAME))
  ?a <- (rl-action (is-selected TRUE) (is-finished TRUE))
  (not (rl-episode-end (success TRUE)))
  =>
  (assert (rl-episode-end (success TRUE)))
  (modify ?a (reward ?*POINTS-EPISODE-END-SUCCESS*))
)

(defrule rl-stop-agent-on-training-end
  (rl-end-training)
=>
  (cx-shutdown)
)