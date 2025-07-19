(defrule workpiece-not-found
    ?at <- (rcll-agent-task (task-id ?task-seq) (robot ?robot)
    (outcome ?outcome&~UNKNOWN) (task-name ?task-name) (retry-count ?count) (executor-id ?ex-id)(error-code ?error-code&~0))
    ?ex <- (executor (id ?ex-id))
    ?cur-task-seq <- (current-rcll-agent-task-id (task-id ?seq) (robot ?robot))
    ?at-list <- (agent-task-list (executor-id ?ex-id) (current-task-id ?seq))
    (test (or (eq ?error-code ?*NO-WORKPIECE-FOUND*) 
                (eq ?error-code ?*NO-WORKPIECE-IN-GRIPPER*))
     )
    =>
    (if (< ?count 3) then
        (bind ?next-seq (+ 1 ?seq))
        (modify ?at (retry-count (+ 1 ?count)) (task-id (+ 1 ?seq)) (outcome UNKNOWN))
        (modify ?cur-task-seq (task-id ?next-seq))
        (modify ?at-list (current-task-id ?next-seq))

    )
    else
    (modify ?ex (state ABORTED))
)