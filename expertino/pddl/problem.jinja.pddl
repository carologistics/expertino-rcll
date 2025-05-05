; Copyright (c) 2024 Carologistics
;
; Licensed under the Apache License, Version 2.0 (the "License");
; you may not use this file except in compliance with the License.
; You may obtain a copy of the License at
;
;     http://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS,
; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; See the License for the specific language governing permissions and
; limitations under the License.

(define (problem workpiece-flow-problem)
  (:domain workpiece-flow)

   (:objects
     pay-token - payment
     grey-carrier - carrier
     black-carrier - carrier
{% block objects %}
{% endblock %}
   )

   (:init
     ;; Machine side mapping
     (in bs bs-input)
     (out bs bs-output)
     (in cs1 cs1-input)
     (out cs1 cs1-output)
     (in cs2 cs2-input)
     (out cs2 cs2-output)
     (in rs1 rs1-input)
     (out rs1 rs1-output)
     (in rs2 rs2-input)
     (out rs2 rs2-output)
     (in ss ss-input)
     (out ss ss-output)
     (in ds ds-input)

     (free bs-input)
     (free bs-output)
     (free cs1-input)
     (free cs1-output)
     (free cs2-input)
     (free cs2-output)
     (free rs1-input)
     (free rs1-output)
     (free rs2-input)
     (free rs2-output)
     (free ss-input)
     (free ss-output)
     (free ds-input)

     (token-step grey-carrier cs1-input cap-grey)
     (token-step grey-carrier cs1-output dispose)
     (token-step black-carrier cs2-input cap-black)
     (token-step black-carrier cs2-output dispose)

     (= (pay-count rs1) 0)
     (= (pay-count rs2) 0)

    (on-shelf grey-carrier cs1)
    ;(next-shelf grey1 grey2)
    ;(next-shelf grey2 grey3)

    (on-shelf black-carrier cs2)
    ;(next-shelf black1 black2)
    ;(next-shelf black2 black3)

    (step-place dispose rs1-slide)
    (step-place dispose rs2-slide)

    (step-place ring-yellow1 rs2-input)
    (step-place ring-yellow2 rs2-input)
    (step-place ring-yellow3 rs2-input)
    (step-place ring-green1 rs1-input)
    (step-place ring-green2 rs1-input)
    (step-place ring-green3 rs1-input)
    (step-place ring-blue1 rs2-input)
    (step-place ring-blue2 rs2-input)
    (step-place ring-blue3 rs2-input)
    (step-place ring-orange1 rs1-input)
    (step-place ring-orange2 rs1-input)
    (step-place ring-orange3 rs1-input)
    (step-place cap-grey cs1-input)
    (step-place cap-black cs2-input)
    (step-place deliver ds-input)

    (can-buffer cs1 cap-grey)
    (can-buffer cs2 cap-black)

    (rs-slide rs1 rs1-slide)
    (rs-slide rs2 rs2-slide)

    ;; Side availability
    (usable bs)
    (usable cs1)
    (usable cs2)
    (usable rs1)
    (usable rs2)
    (usable ss)
    (usable ds)
{% block init %}
{% endblock %}
  )

  (:goal
    (and
{% block goals %}
{% endblock %}
    )
  )
)
