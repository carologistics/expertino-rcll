; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.

(deftemplate refbox-peer
  (slot name (type SYMBOL))
  (slot peer-id (type INTEGER))
)

(defrule refbox-comm-enable-local-public
  "Enable local peer connection to the unencrypted refbox channel"
  (executive-init)
  (not (executive-finalize))
  (confval (path "/clips_executive/parameters/rcll/peer_address") (value ?peer-address))
  (confval (path "/clips_executive/parameters/rcll/peer_send_port") (value ?peer-send-port))
  (confval (path "/clips_executive/parameters/rcll/peer_recv_port") (value ?peer-recv-port))
  (not (refbox-peer (name refbox-public)))
  =>
  (printout t "Enabling local peer (public)" crlf)
  (bind ?peer-id (pb-peer-create-local ?peer-address ?peer-send-port ?peer-recv-port))
  (assert (refbox-peer (name refbox-public) (peer-id ?peer-id)))
)

(defrule refbox-comm-close-local-public
  "Disable the local peer connection on finalize"
  (executive-finalize)
  ?pe <- (refbox-peer (name refbox-public) (peer-id ?peer-id))
  =>
  (printout t "Closing local peer (public)" crlf)
  (pb-peer-destroy ?peer-id)
  (retract ?pe)
)

(defrule refbox-comm-enable-local-team-private
  "Enable local peer connection to the encrypted team channel"
  (executive-init)
  (game-state (team-color ?team-color&~NOT-SET))
  (refbox-peer (name refbox-public))
  (confval (path "/clips_executive/parameters/rcll/peer_address") (value ?address))
  (confval (path "/clips_executive/parameters/rcll/crypto_key") (value ?key))
  (confval (path "/clips_executive/parameters/rcll/cipher") (value ?cipher))
  (confval (path "/clips_executive/parameters/rcll/cyan_recv_port") (value ?cyan-recv-port))
  (confval (path "/clips_executive/parameters/rcll/cyan_send_port") (value ?cyan-send-port))
  (confval (path "/clips_executive/parameters/rcll/magenta_recv_port") (value ?magenta-recv-port))
  (confval (path "/clips_executive/parameters/rcll/magenta_send_port") (value ?magenta-send-port))
  (not (refbox-peer (name refbox-private)))
  =>
  (if (eq ?team-color CYAN)
    then
      (printout t "Enabling local peer (cyan only)" crlf)
      (bind ?peer-id (pb-peer-create-local-crypto ?address ?cyan-send-port ?cyan-recv-port ?key ?cipher))
      else
      (printout t "Enabling local peer (magenta only)" crlf)
      (bind ?peer-id (pb-peer-create-local-crypto ?address ?magenta-send-port ?magenta-recv-port ?key ?cipher))
    )
  (assert (refbox-peer (name refbox-private) (peer-id ?peer-id)))
)

