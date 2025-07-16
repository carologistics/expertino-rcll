(defrule feedback-subtask-success
  ?at-list <- (agent-task-list (executor-id ?id) (tasks ?cur-task $?rest) (current-task-id ?seq))
  (executor (id ?ex-id) (state ACCEPTED) (pddl-action-id ?action-id) (worker ?robot))
  ?at <- (rcll-agent-task (task-id ?seq) (task-name ?cur-task) (outcome ?outcome&~UNKNOWN) (robot ?robot))
  ?cur-task-seq <- (current-rcll-agent-task-id (task-id ?seq) (robot ?robot))
  (pddl-action (id ?action-id) (name ?action&~carrier-to-input) (instance ?instance) (params $?params))
  (feedback (executor-id ?ex-id) (feedback-code ?feedback-code) (recieved-at ?time))
  (test (or
          (eq ?feedback-code ?*SUBTASK-DRIVE-TO-SRC-SUCCESS*)
          (eq ?feedback-code ?*SUBTASK-DRIVE-TO-DEST-SUCCESS*)
          (eq ?feedback-code ?*SUBTASK-PICK-UP-SUCCESS*)
          (eq ?feedback-code ?*SUBTASK-PLACE-DOWN-SUCCESS*)
  ))
  =>
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
    (assert (pddl-action-get-effect (action ?sub-action-id) (effect-type ALL) (apply TRUE)))
    
  )                                                                         
)


(defrule feedback-subtask-success-carrier-to-input
  ?at-list <- (agent-task-list (executor-id ?id) (tasks ?cur-task $?rest) (current-task-id ?seq))
  (executor (id ?ex-id) (state ACCEPTED) (pddl-action-id ?action-id) (worker ?robot))
  ?at <- (rcll-agent-task (task-id ?seq) (task-name ?cur-task) (outcome ?outcome&~UNKNOWN) (robot ?robot))
  ?cur-task-seq <- (current-rcll-agent-task-id (task-id ?seq) (robot ?robot))
  (pddl-action (id ?action-id) (name carrier-to-input) (instance ?instance) (params $?params))
  (feedback (executor-id ?ex-id) (feedback-code ?feedback-code) (recieved-at ?time))
  (test (or
          (eq ?feedback-code ?*SUBTASK-DRIVE-TO-SRC-SUCCESS*)
          (eq ?feedback-code ?*SUBTASK-DRIVE-TO-DEST-SUCCESS*)
          (eq ?feedback-code ?*SUBTASK-PICK-UP-SUCCESS*)
          (eq ?feedback-code ?*SUBTASK-PLACE-DOWN-SUCCESS*)
  ))
  =>
  (if (eq ?cur-task Deliver) then
    (assert (pddl-action-get-effect (action ?action-id) (effect-type END) (apply TRUE))) 
  )
)
