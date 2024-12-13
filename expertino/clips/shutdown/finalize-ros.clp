(defrule finalize-ros-destroy-client
" Delete each client on executive finalize. "
  (executive-finalize)
  (ros-msgs-client (service ?service))
=>
  (printout debug "Destroying client" crlf)
  (ros-msgs-destroy-client ?service)
)

(defrule finalize-ros-destroy-message
" Delete each message on executive finalize. "
  (executive-finalize)
  ?msg-f <- (ros-msgs-message (msg-ptr ?ptr))
=>
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
)
(defrule finalize-ros-destroy-response
" Delete each response on executive finalize. "
  (executive-finalize)
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
=>
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
)
