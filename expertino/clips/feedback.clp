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

(defrule feedback-subtask-failed
  ?at-list <- (agent-task-list (executor-id ?id) (tasks ?cur-task $?rest) (current-task-id ?seq))
  (executor (id ?ex-id) (state ACCEPTED) (pddl-action-id ?action-id) (worker ?robot))
  ?at <- (rcll-agent-task (task-id ?seq) (task-name ?cur-task) (outcome ?outcome&~UNKNOWN) (robot ?robot)(num-retries ?retries))
  ?cur-task-seq <- (current-rcll-agent-task-id (task-id ?seq) (robot ?robot))
  (pddl-action (id ?action-id) (name ?action&~carrier-to-input) (instance ?instance) (params $?params))
  (feedback (executor-id ?ex-id) (feedback-code ?feedback-code) (recieved-at ?time))
  (test (or
          (eq ?feedback-code ?*SUBTASK-DRIVE-TO-SRC-FAILED*)
          (eq ?feedback-code ?*SUBTASK-DRIVE-TO-DEST-FAILED*)
          (eq ?feedback-code ?*SUBTASK-PICK-UP-FAILED*)
          (eq ?feedback-code ?*SUBTASK-PLACE-DOWN-FAILED*)
  ))
  =>
  (bind ?sub-action nil)                                                    
  (switch ?cur-task                                                         
    (case Move-src then   
      (if (< ?retries 3) then
      (bind ?new-seq (+ ?seq 1))
      (bind ?new-retries (+ ?retries 1))
      (retract ?at)
      (retract ?cur-task-seq)
      (assert (rcll-agent-task (task-id ?new-seq) (executor-id ?ex-id) (task-name ?cur-task) (outcome UNKNOWN) (robot ?robot)(num-retries ?new-retries)))
      (assert (current-rcll-agent-task-id (task-id ?new-seq) (robot ?robot)))
      (printout t "Retry: " ?retries " for " ?cur-task " task-id: " ?new-seq crlf)
      )
      
    )
    (case Move-dest then
      (if (< ?retries 3) then
      (bind ?new-seq (+ ?seq 1))
      (bind ?new-retries (+ ?retries 1))
      (retract ?at)
      (retract ?cur-task-seq)
      (assert (rcll-agent-task (task-id ?new-seq) (executor-id ?ex-id) (task-name ?cur-task) (outcome UNKNOWN) (robot ?robot)(num-retries ?new-retries)))
      (assert (current-rcll-agent-task-id (task-id ?new-seq) (robot ?robot)))
      (printout t "Retry: " ?retries " for " ?cur-task " task-id: " ?new-seq crlf)
      )
    )
    (case Retrieve then     
      ;TODO: Implement what if Retrieve fails                                                 
      (printout t "Retrieve FAILED task-id: " ?seq " action-id: " ?action-id crlf)
    )                                                                       
    (case Deliver then                      
    ;TODO: Implement what if Deliver fails                                
      (printout t "Deliver FAILED task-id: " ?seq " action-id: " ?action-id crlf)             
    )
  )
  (if (neq ?sub-action nil)                                                 
   then                   
    (printout t "Action Failed: " ?sub-action crlf)
    
  )
)

;(defrule failed-task-Retrieve
;  (declare (salience 1000))
;  ?at <- (rcll-agent-task (task-id ?seq) (task-name Move-dest) (task-type Move) (outcome UNKNOWN) (robot ?robot)(num-retries ?retries))
;  =>
;  (printout t "MODIFY outcome to FAILED" crlf)
;  (modify ?at (outcome FAILED))
;)