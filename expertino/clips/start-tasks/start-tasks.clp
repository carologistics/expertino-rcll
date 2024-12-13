(deftemplate start-task
  " Establish start routines with defined order of execution.
    @slot name: unique name of the task. Set manually. No modification needed.
    @slot wait-for: list of other start-task names that need to complete before the current one is started. Set manually, modified automatically.
    @slot parts: list of things this task consists of. This list describes the things the task will do once it is active. Hence, user-code is needed to subsequently delete each member of the list in order to achieve the task.
	Set and modified manually.
	@slot state: Set and modified automatically.
	 - WAITING: Initial state of each task, is switched to ACTIVE once all dependent tasks (defined by wait-for) are DONE.
	 - ACTIVE: task is started, user-code needs to fulfill the parts (and subsequently delete each part from the list of parts).
	 - DONE: Once an ACTIVE task has no more parts, it is automatically switched to DONE.
  "
  (slot name (type SYMBOL))
  (multislot wait-for (type SYMBOL) (default (create$)))
  (multislot parts (type SYMBOL) (default (create$)))
  (slot state (type SYMBOL) (allowed-values WAITING ACTIVE DONE) (default WAITING))
)

(defrule start-task-active
  " A start-task is active if there is nothing to wait for anymore."
  ?st <- (start-task (state WAITING) (wait-for))
  =>
  (modify ?st (state ACTIVE))
)

(defrule start-task-wait-for-reduce
  " A start-task can cross-of a task it waits for, once said task is done. "
  (start-task (name ?done-task) (state DONE))
  ?st <- (start-task (state WAITING) (wait-for $?wait-for&:(member$ ?done-task ?wait-for)))
  =>
  (modify ?st (wait-for (delete-member$ ?wait-for ?done-task)))
)

(defrule start-task-done
  " A start-task is done once it has no parts to work on."
  ?st <- (start-task (state ACTIVE) (parts))
  =>
  (modify ?st (state DONE))
)
