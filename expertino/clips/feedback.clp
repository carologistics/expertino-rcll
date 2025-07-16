(defrule feedback-subtask-success
  (executor (id ?ex-id) (pddl-action-id ?action) (state ACCEPTED))
  (pddl-action (id ?action) (name ?action-name) (params $?params) (instance ?instance))
  (feedback (executor-id ?ex-id) (feedback-code ?feedback-code) (received-at ?time))
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
     (if (and (eq ?action carrier-to-input) (eq ?cur-task Deliver)) then
         (assert (pddl-action-get-effect (action ?action) (effect-type END) (apply TRUE))) 
       else
         (bind ?sub-action-id (gensym*))                                        
         (assert (pddl-action (id ?sub-action-id) (name ?sub-action) (instance ?instance) (params ?params)))
         (assert (pddl-action-get-effect (action ?sub-action-id) (effect-type ALL) (apply TRUE)))
     )
  )                                                                         
)



