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
     wp1 wp2 - product
     pay1 pay2 pay3 pay4 pay5 pay6 pay7 pay8 pay9 pay10 pay11 pay12 - payment
     grey1 grey2 grey3 - carrier
     black1 black2 black3 - carrier
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

     (free rs1-slide)
     (free rs2-slide)

     (spawnable wp1)
     (spawnable wp2)
     (spawnable pay1)
     (spawnable pay2)
     (spawnable pay3)

     (next-payment pay1 pay4)
     (next-payment pay2 pay5)
     (next-payment pay3 pay6)
     (next-payment pay4 pay7)
     (next-payment pay5 pay8)
     (next-payment pay6 pay9)
     (next-payment pay7 pay10)
     (next-payment pay8 pay11)
     (next-payment pay9 pay12)

     (step pay1 dispose)
     (step pay2 dispose)
     (step pay3 dispose)
     (step pay4 dispose)
     (step pay5 dispose)
     (step pay6 dispose)
     (step pay7 dispose)
     (step pay8 dispose)
     (step pay9 dispose)
     (step pay10 dispose)
     (step pay11 dispose)
     (step pay12 dispose)

     (step grey1 cap-grey)
     (next-step grey1 cap-grey dispose)
     (step grey2 cap-grey)
     (next-step grey2 cap-grey dispose)
     (step grey3 cap-grey)
     (next-step grey3 cap-grey dispose)
     (step black1 cap-black)
     (next-step black1 cap-black dispose)
     (step black2 cap-black)
     (next-step black2 cap-black dispose)
     (step black3 cap-black)
     (next-step black3 cap-black dispose)
     (usable grey1)
     (usable grey2)
     (usable grey3)
     (usable black1)
     (usable black2)
     (usable black3)

     (= (order) 0)
     (= (pay-count rs1) 0)
     (= (pay-count rs2) 0)
     (= (price ring-yellow1) 1)
     (= (price ring-yellow2) 1)
     (= (price ring-yellow3) 1)
     (= (price ring-blue1) 2)
     (= (price ring-blue2) 2)
     (= (price ring-blue3) 2)
     (= (price ring-orange1) 0)
     (= (price ring-orange2) 0)
     (= (price ring-orange3) 0)
     (= (price ring-green1) 0)
     (= (price ring-green2) 0)
     (= (price ring-green3) 0)

    (on-shelf grey1 cs1)
    (on-shelf grey2 cs1); SHELF-MID)
    (on-shelf grey3 cs1); SHELF-RIGHT)

    (on-shelf black1 cs2)
    (on-shelf black2 cs2); SHELF-MID)
    (on-shelf black3 cs2); SHELF-RIGHT)

    (step wp1 base-red)
    (next-step wp1 base-red ring-blue1)
    (next-step wp1 ring-blue1 ring-yellow2)
    (next-step wp1 ring-yellow2 ring-blue3)
    (next-step wp1 ring-blue3 cap-grey)
    (next-step wp1 cap-grey deliver)
    (next-step wp1 deliver done)

    (step wp2 base-black)
    (next-step wp2 base-black ring-green1)
    (next-step wp2 ring-green1 ring-yellow2)
    (next-step wp2 ring-yellow2 ring-orange3)
    (next-step wp2 ring-orange3 cap-black)
    (next-step wp2 cap-black deliver)
    (next-step wp2 deliver done)

    (step-place dispose rs1-slide)
    (step-place dispose rs2-slide)

    (step-place ring-yellow1 rs1-input)
    (step-place ring-yellow2 rs1-input)
    (step-place ring-yellow3 rs1-input)
    (step-place ring-green1 rs1-input)
    (step-place ring-green2 rs1-input)
    (step-place ring-green3 rs1-input)
    (step-place ring-blue1 rs2-input)
    (step-place ring-blue2 rs2-input)
    (step-place ring-blue3 rs2-input)
    (step-place ring-orange1 rs2-input)
    (step-place ring-orange2 rs2-input)
    (step-place ring-orange3 rs2-input)
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
  )

  (:goal
    (and
      (step wp1 done)
      ;(at grey3 rs1-slide)
      ;(buffered cs1 cap-grey)
      ;(buffered cs2 cap-black)
      ;(= (pay-count rs1) 1)
      ;(= (pay-count rs2) 0)
      ;(at wp2 rs1-input)
      ;(at pay2 rs1-slide)
      (step wp2 done)
    )
  )
)
