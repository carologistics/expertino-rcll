;---------------------------------------------------------------------------
;  refbox-comm-init.clp - Initialize RefBox communication
;
;  Copyright  2023  TariK Viehmann <viehmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Library General Public License for more details.
;
; Read the full text in the LICENSE.GPL file in the doc directory.
;
(deftemplate refbox-peer
  (slot name (type SYMBOL))
  (slot peer-id (type INTEGER))
)

(defrule refbox-comm-enable-public
  "Enable peer connection to the unencrypted refbox channel"
  ; (declare (salience ?*PRIORITY-LOW*))
  (executive-init)
  (not (executive-finalize))
  (confval (path "/clips_executive/parameters/rcll/peer-address") (value ?peer-address))
  (confval (path "/clips_executive/parameters/rcll/peer-port") (value ?peer-port&~0))
  (not (refbox-peer (name refbox-public)))
  =>
  (printout t "Enabling remote peer (public)" crlf)
  (bind ?peer-id (pb-peer-create ?peer-address ?peer-port))
  (assert (refbox-peer (name refbox-public) (peer-id ?peer-id)))
)

(defrule refbox-comm-enable-local-public
  "Enable local peer connection to the unencrypted refbox channel"
  (executive-init)
  (not (executive-finalize))
  (confval (path "/clips_executive/parameters/rcll/peer-address") (value ?peer-address))
  (confval (path "/clips_executive/parameters/rcll/peer-send-port") (value ?peer-send-port))
  (confval (path "/clips_executive/parameters/rcll/peer-recv-port") (value ?peer-recv-port))
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
  (team-color ?team-color)
  (refbox-peer (name refbox-public))
  (confval (path "/clips_executive/parameters/rcll/peer-address") (value ?address))
  (confval (path "/clips_executive/parameters/rcll/crypto-key") (value ?key))
  (confval (path "/clips_executive/parameters/rcll/cipher") (value ?cipher))
  (confval (path "/clips_executive/parameters/rcll/cyan-recv-port") (value ?cyan-recv-port))
  (confval (path "/clips_executive/parameters/rcll/cyan-send-port") (value ?cyan-send-port))
  (confval (path "/clips_executive/parameters/rcll/magenta-recv-port") (value ?magenta-recv-port))
  (confval (path "/clips_executive/parameters/rcll/magenta-send-port") (value ?magenta-send-port))
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

(defrule refbox-comm-enable-team-private
  "Enable local peer connection to the encrypted team channel"
  (executive-init)
  (team-color ?team-color)
  (refbox-peer (name refbox-public))
  (confval (path "/clips_executive/parameters/rcll/peer-address") (value ?address))
  (confval (path "/clips_executive/parameters/rcll/crypto-key") (value ?key))
  (confval (path "/clips_executive/parameters/rcll/cipher") (value ?cipher))
  (confval (path "/clips_executive/parameters/rcll/cyan-port") (value ?cyan-port&~0))
  (confval (path "/clips_executive/parameters/rcll/magenta-port") (value ?magenta-port&~0))
  (not (refbox-peer (name refbox-private)))
  =>
  (if (eq ?team-color CYAN)
    then
      (printout t "Enabling remote peer (cyan only)" crlf)
      (bind ?peer-id (pb-peer-create-crypto ?address ?cyan-port ?key ?cipher))
      else
      (printout t "Enabling remote peer (magenta only)" crlf)
      (bind ?peer-id (pb-peer-create-crypto ?address ?magenta-port ?key ?cipher))
    )
  (assert (refbox-peer (name refbox-private) (peer-id ?peer-id)))
)
