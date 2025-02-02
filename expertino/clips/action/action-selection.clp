; the action selection picks one action among the actions that are currently on the 
; agenda and selects the next action to execute. The selection is based on
; the priority values of the agenda items. 
; Only one agenda item is selected at a time.


(defrule action-agenda-item-selection
    (not (agenda-action-item (action ?some-action-id) (plan ?plan-id) (execution-state SELECTED)))
    ?action <- (agenda-action-item (action ?action-id) (plan ?plan-id) (priority $?priority) (execution-state PENDING))
    (not
        (and
            (agenda-action-item (action ?other-action-id) (plan ?plan-id) (priority $?other-priority) (execution-state PENDING))
            (test (> (+ (expand$ ?priority)) (+ (expand$ ?other-priority))))
        )
    )
    (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
    =>
    (modify ?action (execution-state SELECTED))
    (printout green "Selected action " ?action-name "[" ?action-id "]" ?action-params " from agenda" crlf)
)

(defrule action-agenda-item-update-selection
    ?action <- (agenda-action-item (action ?action-id) (plan ?plan-id) (priority $?priority) (execution-state SELECTED))
    ?other-action <- (agenda-action-item (action ?other-action-id) (plan ?plan-id) (priority $?other-priority) (execution-state PENDING))
    (test (> (+ (expand$ ?priority)) (+ (expand$ ?other-priority))))
    (pddl-action (id ?other-action-id) (name ?action-name) (params $?action-params))
    =>
    (modify ?action (execution-state PENDING))
    (printout yellow "Deselected action " ?action-id " from agenda" crlf)
    (modify ?other-action (execution-state SELECTED))
    (printout green "Selected action " ?action-name "[" ?other-action-id "]" ?action-params " from agenda" crlf)
)



