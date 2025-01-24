; the action selection picks one action among the actions that are currently on the 
; agenda and selects the next action to execute. The selection is based on
; the priority values of the agenda items. 
; Only one agenda item is selected at a time.


(defrule action-agenda-item-selection
    (not (agenda-action-item (action ?action-id) (plan ?plan-id) (execution-state SELECTED)))
    ?action <- (agenda-action-item (action ?action-id) (plan ?plan-id) (priority $?priority) (execution-state PENDING))
    (not
        (and
            (agenda-action-item (action ?other-action-id) (plan ?plan-id) (priority $?other-priority) (execution-state PENDING))
            (test (> (+ (expand$ ?priority)) (+ (expand$ ?other-priority))))
        )
    )
    =>
    (modify ?action (state SELECTED))
)

(defrule action-agenda-item-update-selection
    ?action <- (agenda-action-item (action ?action-id) (plan ?plan-id) (priority $?priority) (execution-state SELECTED))
    ?other-action (agenda-action-item (action ?other-action-id) (plan ?plan-id) (priority $?other-priority) (execution-state PENDING))
    (test (> (+ (expand$ ?priority)) (+ (expand$ ?other-priority))))
    =>
    (modify ?action (state PENDING))
    (modify ?other-action (state SELECTED))
)



