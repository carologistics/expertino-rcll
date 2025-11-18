(deftemplate reset-goal
    (slot goal (type EXTERNAL-ADDRESS))
)

(defrule reset-cx-stage-pre-reset
    (reset-cx (stage PRE-RESET))
    (expertino-rl-interfaces-stop-refbox-client (server ?server&:(eq ?server "cx_rl_node/stop_refbox")))
    (not (reset-goal (goal ?g)))
    =>
    (bind ?goal (expertino-rl-interfaces-stop-refbox-goal-create))
    (assert (reset-goal (goal ?goal)))
    (expertino-rl-interfaces-stop-refbox-send-goal ?goal ?server)
)

(defrule reset-cx-stage-pre-reset-finished
    ?r <- (reset-cx (stage PRE-RESET))
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
    (modify ?r (stage RESET))
    (expertino-rl-interfaces-stop-refbox-result-destroy ?res-ptr)
    (retract ?wr)
    (expertino-rl-interfaces-stop-refbox-client-goal-handle-destroy ?ghp)
    (retract ?f)
    (expertino-rl-interfaces-stop-refbox-goal-destroy ?goal)
    (retract ?rg)
)

(defrule reset-cx-stage-post-reset
    (reset-cx (stage POST-RESET))
    (expertino-rl-interfaces-start-refbox-client (server ?server&:(eq ?server "cx_rl_node/start_refbox")))
    (not (reset-goal (goal ?g)))
    =>
    (bind ?goal (expertino-rl-interfaces-start-refbox-goal-create))
    (assert (reset-goal (goal ?goal)))
    (expertino-rl-interfaces-start-refbox-send-goal ?goal ?server)
)

(defrule reset-cx-stage-post-reset-finished
    ?r <- (reset-cx (stage POST-RESET))
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
    (modify ?r (stage FINALIZE))
    (expertino-rl-interfaces-start-refbox-result-destroy ?res-ptr)
    (retract ?wr)
    (bind ?g-id (expertino-rl-interfaces-start-refbox-client-goal-handle-get-goal-id ?ghp))
    (expertino-rl-interfaces-start-refbox-client-goal-handle-destroy ?ghp)
    (retract ?f)
    (expertino-rl-interfaces-start-refbox-goal-destroy ?goal)
    (retract ?rg)
)