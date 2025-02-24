(defrule agent-task-list-create-transport
  ?ex <- (executor (id ?ex-id) (state INIT) (worker ?robot) (pddl-action-id ?action-id))
  (worker (id ?robot) (type ROBOT))
  (confval (path "/rcll-simulator/enabled") (value TRUE))
  (protobuf-peer (name ?robot) (peer-id ?peer-id))
  (not (agent-task-list (executor-id ?ex-id)))
  ?pa <- (pddl-action (id ?action-id) (name transport|transport-to-slide) (params ?wp ?from ?to $?))
  (game-state (team-color ?team-color))
  =>
  (bind ?from-mps (pddl-place-to-refbox-mps ?from ?team-color))
  (bind ?from-side (pddl-place-to-mps-side ?from))
  (bind ?to-mps (pddl-place-to-refbox-mps ?to ?team-color))
  (bind ?to-side (pddl-place-to-mps-side ?to))
  (assert (agent-task-list (id (sym-cat TASK-LIST-(gensym*))) (executor-id ?ex-id) (pddl-action-id ?action-id)
               (tasks Move-src Retrieve Move-dest Deliver) 
               (params ?wp ?from-mps ?from-side ?to-mps ?to-side)))
  (modify ?ex (state REQUESTED))
)

(defrule agent-task-list-create-carrier-to-input
  ?ex <- (executor (id ?ex-id) (state INIT) (worker ?robot) (pddl-action-id ?action-id))
  (worker (id ?robot) (type ROBOT))
  (confval (path "/rcll-simulator/enabled") (value TRUE))
  (protobuf-peer (name ?robot) (peer-id ?peer-id))
  (not (agent-task-list (executor-id ?ex-id)))
  ?pa <- (pddl-action (id ?action-id) (name carrier-to-input) 
           (params ?wp ?next-carr ?step ?mps ?to-place))
  (game-state (team-color ?team-color))
  ;TODO check for cap carrier on CS shelf
  =>
  (bind ?from-mps (pddl-place-to-refbox-mps ?mps ?team-color))
  (bind ?from-side SHELF)
  (bind ?to-side (pddl-place-to-mps-side ?to-place))
  (assert (agent-task-list (id (sym-cat TASK-LIST-(gensym*))) (executor-id ?ex-id) (pddl-action-id ?action-id)
              (tasks Move-src Retrieve Move-dest Deliver) 
              (params ?wp ?from-mps ?from-side ?from-mps ?to-side)))
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

(defrule agent-task-list-delete-active
  ?at-list <- (agent-task-list (executor-id ?id) (tasks ?cur-task $?rest) (current-task-id ?seq))
  (executor (id ?id) (state ACCEPTED) (pddl-action-id ?action-id))
  ?at <- (rcll-agent-task (task-id ?seq) (task-name ?cur-task) (outcome SUCCEEDED) (robot ?robot))
  ?cur-task-seq <- (current-rcll-agent-task-id (task-id ?seq) (robot ?robot))
  (pddl-action (id ?action-id) (name ?action) (instance ?instance) (params $?params))
  =>
  (modify ?at-list (tasks $?rest))
  (modify ?cur-task-seq (task-id (+ 1 ?seq)))
  (if (or (eq ?action transport) (eq ?action transport-to-slide))
   then
     (bind ?sub-action nil)
     (switch ?cur-task
       (case Move-src then
         (bind ?sub-action (sym-cat ?action -step-1-drive-to))
       )
       (case Retrieve then
         (bind ?sub-action (sym-cat ?action -step-2-pick-up))
       )
       (case Deliver then 
         (bind ?sub-action (sym-cat ?action -step-4-place-down))
       )
     )
     (if (neq ?sub-action nil)
      then
        (bind ?sub-action-id (gensym*)) 
        (assert (pddl-action (id ?sub-action-id) (name ?sub-action) (instance ?instance) (params ?params)))
        (assert (pddl-action-apply-effect (action ?sub-action-id) (effect-type ALL)))
     )
  ) 
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
     (if (eq ?action-name carrier-to-input)
      then
        (assert (pddl-action-apply-effect (action ?action-id) (effect-type END))) 
     )
   else
     (modify ?ex (state ABORTED))
  )
)
