(deftemplate reset-goal
    (slot goal (type EXTERNAL-ADDRESS))
)

(defrule reset-cx-stage-pre-reset
    (rl-reset-env (state USER-CLEANUP))
    (expertino-rl-interfaces-stop-refbox-client (server ?server&:(eq ?server "cx_rl_node/stop_refbox")))
    (not (reset-goal (goal ?g)))
    =>
    (bind ?goal (expertino-rl-interfaces-stop-refbox-goal-create))
    (assert (reset-goal (goal ?goal)))
    (expertino-rl-interfaces-stop-refbox-send-goal ?goal ?server)
    (save-facts reset-protobuf-peers local protobuf-peer)
)

(defrule reset-cx-stage-pre-reset-finished
    ?r <- (rl-reset-env (state USER-CLEANUP))
    ?rg <- (reset-goal (goal ?goal))
    ?f <- (expertino-rl-interfaces-stop-refbox-goal-response (server "cx_rl_node/stop_refbox") (client-goal-handle-ptr ?ghp))
    ?wr <- (expertino-rl-interfaces-stop-refbox-wrapped-result (server "cx_rl_node/stop_refbox") (goal-id ?uuid) (code SUCCEEDED) (result-ptr ?res-ptr))
    =>
    (bind ?stop-completed (expertino-rl-interfaces-stop-refbox-result-get-field ?res-ptr "success"))
    (if ?stop-completed then
        (printout green "stopping refbox successful" crlf)
    else
        (printout error "stopping unsuccessful" crlf)
    )
    (modify ?r (state LOAD-FACTS))
    (expertino-rl-interfaces-stop-refbox-result-destroy ?res-ptr)
    (retract ?wr)
    (expertino-rl-interfaces-stop-refbox-client-goal-handle-destroy ?ghp)
    (retract ?f)
    (expertino-rl-interfaces-stop-refbox-goal-destroy ?goal)
    (retract ?rg)
)

(defrule reset-cx-stage-post-reset
    (rl-reset-env (state USER-INIT))
    (expertino-rl-interfaces-start-refbox-client (server ?server&:(eq ?server "cx_rl_node/start_refbox")))
    (not (reset-goal (goal ?g)))
    =>
    (bind ?goal (expertino-rl-interfaces-start-refbox-goal-create))
    (assert (reset-goal (goal ?goal)))
    (expertino-rl-interfaces-start-refbox-send-goal ?goal ?server)
    (do-for-all-facts ((?pi pddl-instance))
        TRUE
        (retract ?pi))
    (do-for-all-facts ((?pf pddl-fluent))
        TRUE
        (retract ?pf))
    (do-for-all-facts ((?pnf pddl-numeric-fluent))
        TRUE
        (retract ?pnf))
    (load-facts reset-protobuf-peers)
    (assert (last-game-time (game-time 0.) (last-time (time))))
)

(defrule reset-cx-stage-post-reset-finished
    ?r <- (rl-reset-env (state USER-INIT))
    ?rg <- (reset-goal (goal ?goal))
    ?f <- (expertino-rl-interfaces-start-refbox-goal-response (server "cx_rl_node/start_refbox") (client-goal-handle-ptr ?ghp))
    ?wr <- (expertino-rl-interfaces-start-refbox-wrapped-result (server "cx_rl_node/start_refbox") (goal-id ?uuid) (code SUCCEEDED) (result-ptr ?res-ptr))
    =>
    (bind ?start-completed (expertino-rl-interfaces-start-refbox-result-get-field ?res-ptr "success"))
    (if ?start-completed then
        (printout green "starting refbox successful" crlf)
    else
        (printout error "starting refbox unsuccessful" crlf)
    )
    (modify ?r (state DONE))
    (expertino-rl-interfaces-start-refbox-result-destroy ?res-ptr)
    (retract ?wr)
    (bind ?g-id (expertino-rl-interfaces-start-refbox-client-goal-handle-get-goal-id ?ghp))
    (expertino-rl-interfaces-start-refbox-client-goal-handle-destroy ?ghp)
    (retract ?f)
    (expertino-rl-interfaces-start-refbox-goal-destroy ?goal)
    (retract ?rg)
)