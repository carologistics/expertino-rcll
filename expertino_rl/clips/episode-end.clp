(defrule episode-end
  (declare (salience 1))
  (game-state (phase POST_GAME))
  (not (rl-episode-end (success TRUE)))
  =>
  (bind ?delivery-count 0)
  (do-for-all-facts ((?pf pddl-fluent))
    (and  (eq ?pf:name step)
          (eq (nth$ 2 ?pf:params) done))
    (bind ?delivery-count (+ ?delivery-count 1))
  )
  (printout green "EPISODE-END: Number of deliveries " ?delivery-count crlf)
  (if (eq ?delivery-count 3) then
    (assert (rl-episode-end (success TRUE)))
  else
    (assert (rl-episode-end (success FALSE)))
  )
)

(defrule episode-end-stop-rl-action
  (declare (salience -1))
  (rl-episode-end (success ?success) (reset-triggered FALSE))
  (not (rl-reset-env))
  ?ra <- (rl-action (is-selected TRUE) (is-finished FALSE))
  =>
  (if (eq ?success TRUE) then
    (modify ?ra (is-finished TRUE) (reward ?*CX-RL-REWARD-EPISODE-SUCCESS*))
  else
    (modify ?ra (is-finished TRUE) (reward ?*CX-RL-REWARD-EPISODE-FAILURE*))
  )
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