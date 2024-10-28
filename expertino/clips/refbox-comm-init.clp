; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.

(defrule refbox-comm-enable-local-public
  "Enable local peer connection to the unencrypted refbox channel"
  (not (executive-finalize))
  (confval (path "/game/parameters/rcll/peer_address") (value ?peer-address))
  (confval (path "/game/parameters/rcll/peer_send_port") (value ?peer-send-port))
  (confval (path "/game/parameters/rcll/peer_recv_port") (value ?peer-recv-port))
  (not (protobuf-peer (name refbox-public)))
  =>
  (printout info "Enabling local peer (public) " ?peer-address " " ?peer-send-port " " ?peer-recv-port crlf)
  (bind ?peer-id (pb-peer-create-local ?peer-address ?peer-send-port ?peer-recv-port))
  (assert (protobuf-peer (name refbox-public) (peer-id ?peer-id)))
)

(defrule refbox-comm-enable-local-team-private
  "Enable local peer connection to the encrypted team channel"
  (game-state (team-color ?team-color&~NOT-SET))
  (protobuf-peer (name refbox-public))
  (confval (path "/game/parameters/rcll/peer_address") (value ?address))
  (confval (path "/game/parameters/rcll/crypto_key") (value ?key))
  (confval (path "/game/parameters/rcll/cipher") (value ?cipher))
  (confval (path "/game/parameters/rcll/cyan_recv_port") (value ?cyan-recv-port))
  (confval (path "/game/parameters/rcll/cyan_send_port") (value ?cyan-send-port))
  (confval (path "/game/parameters/rcll/magenta_recv_port") (value ?magenta-recv-port))
  (confval (path "/game/parameters/rcll/magenta_send_port") (value ?magenta-send-port))
  (not (protobuf-peer (name refbox-private)))
  =>
  (if (eq ?team-color CYAN)
    then
      (printout info "Enabling local peer (cyan only)" ?address " " ?cyan-send-port " " ?cyan-recv-port " " ?key " " ?cipher crlf)
      (bind ?peer-id (pb-peer-create-local-crypto ?address ?cyan-send-port ?cyan-recv-port ?key ?cipher))
      else
      (printout info "Enabling local peer (magenta only)" crlf)
      (bind ?peer-id (pb-peer-create-local-crypto ?address ?magenta-send-port ?magenta-recv-port ?key ?cipher))
    )
  (assert (protobuf-peer (name refbox-private) (peer-id ?peer-id)))
)

(defrule refbox-comm-close-peers
  "Disable the peer connections"
  (executive-finalize)
  ?pe <- (protobuf-peer (name ?name) (peer-id ?peer-id))
  =>
  (printout info "Closing peer " ?name crlf)
  (pb-peer-destroy ?peer-id)
  (retract ?pe)
)
