(defrule agent-task-list-create
  ?ex <- (executor (id ?ex-id) (state INIT) (worker ?robot) (pddl-action-id ?action-id))
  (worker (id ?robot) (type ROBOT))
  (confval (path "/rcll-simulator/enabled") (value TRUE))
  (protobuf-peer (name ?robot) (peer-id ?peer-id))
  (not (agent-task-list (executor-id ?ex-id)))
  ?pa <- (pddl-action (id ?action-id) 
           (name ?action-name&transport|transport-to-slide|carrier-to-input|base-transport|pay-with-carrier)
           (params $?action-params))
  (game-state (team-color ?team-color))
  =>
  (bind ?wp (nth$ 1 ?action-params))
  (switch ?action-name
    (case carrier-to-input then
      (bind ?from-mps (pddl-place-to-refbox-mps (nth$ 3 ?action-params) ?team-color))
      (bind ?from-side SHELF)
      (bind ?to-side (pddl-place-to-mps-side (nth$ 4 ?action-params)))
      (bind ?to-mps ?from-mps)
    )
    (case pay-with-carrier then
      (bind ?from-mps (pddl-place-to-refbox-mps (nth$ 3 ?action-params) ?team-color))
      (bind ?from-side (pddl-place-to-mps-side (nth$ 3 ?action-params)))
      (bind ?to-mps (pddl-place-to-refbox-mps (nth$ 4 ?action-params) ?team-color))
      (bind ?to-side (pddl-place-to-mps-side (nth$ 4 ?action-params)))
    )
    (default
      (bind ?from-mps (pddl-place-to-refbox-mps (nth$ 2 ?action-params) ?team-color))
      (bind ?from-side (pddl-place-to-mps-side (nth$ 2 ?action-params)))
      (bind ?to-mps (pddl-place-to-refbox-mps (nth$ 3 ?action-params) ?team-color))
      (bind ?to-side (pddl-place-to-mps-side (nth$ 3 ?action-params)))
    )
  )
  (assert (agent-task-list (id (sym-cat TASK-LIST-(gensym*))) (executor-id ?ex-id) (pddl-action-id ?action-id)
               (tasks Move-src Retrieve Move-dest Deliver Move-away) 
               (params ?wp ?from-mps ?from-side ?to-mps ?to-side)))
  (modify ?ex (state REQUESTED))
)

(defrule agent-task-create-retrieve
  ?at-list <- (agent-task-list (executor-id ?ex-id) (tasks ?task&Retrieve $?rest) 
                          (params $?params))
  ?ex <- (executor (id ?ex-id) (worker ?robot))
  (current-rcll-agent-task-id (robot ?robot) (task-id ?seq)) 
  (not (rcll-agent-task (robot ?robot) (task-id ?seq)))
  =>
  (bind ?mps (nth$ 2 ?params))
  (bind ?mps-side (nth$ 3 ?params))
  (assert (rcll-agent-task (task-id ?seq) (task-name ?task) (robot ?robot) (task-type Retrieve)     
       ;TODO add workpiece information
                (machine ?mps) (side ?mps-side) (executor-id ?ex-id)))
  (modify ?at-list (current-task-id ?seq))
)    

(defrule agent-task-create-deliver
  ?at-list <- (agent-task-list (id ?id) (executor-id ?ex-id) (tasks ?task&Deliver $?rest)
                          (params $?params))
  ?ex <- (executor (id ?ex-id) (worker ?robot))
  (current-rcll-agent-task-id (robot ?robot) (task-id ?seq)) 
  (not (rcll-agent-task (robot ?robot) (task-id ?seq)))
  =>
  (bind ?mps (nth$ 4 ?params))
  (bind ?mps-side (nth$ 5 ?params))
  (assert (rcll-agent-task (task-id ?seq) (task-name ?task) (robot ?robot) (task-type Deliver)     
   ;TODO add workpiece information
   (machine ?mps) (side ?mps-side) (executor-id ?ex-id)                                                              
  ))
  (modify ?at-list (current-task-id ?seq))
)    

(defrule agent-task-create-move
  ?at-list <- (agent-task-list (id ?id) (executor-id ?ex-id) (tasks ?task&Move-src|Move-dest $?rest)
                          (params $?params))
  ?ex <- (executor (id ?ex-id) (worker ?robot))
  (current-rcll-agent-task-id (robot ?robot) (task-id ?seq)) 
  (not (rcll-agent-task (robot ?robot) (task-id ?seq)))
  =>
  ;TODO test for current location to be not equal to desired location
  (if (eq ?task Move-src)
   then 
     (bind ?mps (nth$ 2 ?params))
     (bind ?mps-side (nth$ 3 ?params))
   else 
     (bind ?mps (nth$ 4 ?params))
     (bind ?mps-side (nth$ 5 ?params))
  )
  (assert (rcll-agent-task (task-id ?seq) (task-name ?task) (robot ?robot) (task-type Move)
   (machine ?mps) (side ?mps-side) (executor-id ?ex-id)
  ))
  (modify ?at-list (current-task-id ?seq))
)    

(defrule agent-task-create-move-away
  ?at-list <- (agent-task-list (id ?id) (executor-id ?ex-id) (tasks ?task&Move-away $?rest)
                          (params $?params))
  ?ex <- (executor (id ?ex-id) (worker ?robot))
  (current-rcll-agent-task-id (robot ?robot) (task-id ?seq)) 
  (not (rcll-agent-task (robot ?robot) (task-id ?seq)))
  =>
  (bind ?zone M_Z71)
  (assert (rcll-agent-task (task-id ?seq) (task-name ?task) (robot ?robot) (task-type Move)
   (waypoint ?zone) (executor-id ?ex-id)
  ))
  (modify ?at-list (current-task-id ?seq))
)

(defrule agent-task-collect-feedback
  ;TODO add feedback mechanism for the cancelled AgentTask case
  ?at-list <- (agent-task-list (executor-id ?id) (tasks ?cur-task $?rest) (current-task-id ?seq))
  (executor (id ?ex-id) (state ACCEPTED) (pddl-action-id ?action-id) (worker ?robot))
  ?at <- (rcll-agent-task (task-id ?seq) (task-name ?cur-task) (outcome ?outcome&~UNKNOWN) (robot ?robot))
  ?cur-task-seq <- (current-rcll-agent-task-id (task-id ?seq) (robot ?robot))
  (pddl-action (id ?action-id) (name ?action) (instance ?instance) (params $?params))
  =>
  (switch ?outcome
    (case FAILED then
      (switch ?cur-task
        (case Move-src then
          (bind ?feedback-code ?*SUBTASK-DRIVE-TO-SRC-FAILED*)
        )
        (case Retrieve then
          (bind ?feedback-code ?*SUBTASK-PICK-UP-FAILED*)
        )
        (case Move-dest then
          (bind ?feedback-code ?*SUBTASK-DRIVE-TO-DEST-FAILED*)
        )
        (case Deliver then 
          (bind ?feedback-code ?*SUBTASK-PLACE-DOWN-FAILED*)
        )
      )
    )
    (case SUCCEEDED then
      (switch ?cur-task
        (case Move-src then
          (bind ?feedback-code ?*SUBTASK-DRIVE-TO-SRC-SUCCESS*)
        )
        (case Retrieve then
          (bind ?feedback-code ?*SUBTASK-PICK-UP-SUCCESS*)
        )
        (case Move-dest then
          (bind ?feedback-code ?*SUBTASK-DRIVE-TO-DEST-SUCCESS*)
        )
        (case Deliver then 
          (bind ?feedback-code ?*SUBTASK-PLACE-DOWN-SUCCESS*)
        )
      )
    )
    (case CANCELLED then
       (printout debug "The task " ?seq  " has been cancelled!" crlf) 
    )
  )
  (assert (feedback (id (sym-cat FEEDBACK- gensym*)) (executor-id ?ex-id) (feedback-code ?feedback-code) (recieved-at (now)))) 
)

(defrule agent-task-list-delete-active
  ?at-list <- (agent-task-list (executor-id ?id) (tasks ?cur-task $?rest) (current-task-id ?seq))
  (executor (id ?id) (state ACCEPTED) (pddl-action-id ?action-id) (worker ?robot))
  ?at <- (rcll-agent-task (task-id ?seq) (task-name ?cur-task) (outcome SUCCEEDED) (robot ?robot))
  ?cur-task-seq <- (current-rcll-agent-task-id (task-id ?seq) (robot ?robot))
  (pddl-action (id ?action-id) (name ?action) (instance ?instance) (params $?params))
  =>
  (modify ?at-list (tasks $?rest))
  (modify ?cur-task-seq (task-id (+ 1 ?seq)))
)

(defrule agent-task-list-mark-done
  ?at-list <- (agent-task-list (tasks) (executor-id ?ex-id) (current-task-id ?seq))
  ?ex <- (executor (id ?ex-id) (state ACCEPTED) (pddl-action-id ?action-id))
  (rcll-agent-task (task-id ?seq) (outcome ?outcome&~UNKNOWN))
  (pddl-action (id ?action-id) (name ?action-name) (params $?params))
  =>
  (if (eq ?outcome SUCCEEDED)
   then
     (modify ?ex (state SUCCEEDED))
   else
     (modify ?ex (state ABORTED))
  )
)
