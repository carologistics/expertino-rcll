(defrule unwatch-spam
=>
  ;(unwatch rules plan-update-plan-status)
  (unwatch rules protobuf-cleanup-message)
  (unwatch facts protobuf-msg game-time)
)
