(defglobal
  ?*TOTAL-PRODUCTION-THRESHOLD* = 2
)

(defrule production-strategy-init-filter
  (not (production-strategy-order-filter (name possible-orders)))
  =>
  (assert 
    (production-strategy-order-filter (name possible-orders))
    (production-strategy-order-filter (name active-orders))
    (production-strategy-order-filter (name selected-orders))
  )
)

(defrule production-strategy-add-possible-order
  "add an order to this filter if it has not been fulfilled and the delivery window is ahead"
  ?filter <- (production-strategy-order-filter (name possible-orders) (orders $?possible-orders))
  (order (id ?order-id) (quantity-requested ?q-req) (quantity-delivered ?q-del&:(< ?q-del ?q-req))
          (delivery-end ?end))
  (production-strategy-order-filter (name active-orders) (orders $?active-orders))
  (production-strategy-order-filter (name selected-orders) (orders $?selected-orders))
  (game-time ?gt)
  (test (not (member$ ?order-id ?possible-orders)))
  (test (not (member$ ?order-id ?active-orders)))
  (test (not (member$ ?order-id ?selected-orders)))
  (test (< ?gt ?end))
  =>
  (modify ?filter (orders $?possible-orders ?order-id))
)

(defrule production-strategy-remove-possible-order
  "remove an order from this filter if it has been started, selected, fulfilled, or delivery window has ended"
  (order (id ?order-id) (quantity-requested ?q-req) (quantity-delivered ?q-del) (delivery-end ?end))
  (game-time ?gt)
  ?filter <- (production-strategy-order-filter (name possible-orders) (orders $?possible-orders&:(member$ ?order-id ?possible-orders)))
  (production-strategy-order-filter (name active-orders) (orders $?active-orders))
  (or
    (test (member$ ?order-id ?active-orders))
    (test (= ?q-req ?q-del))
    (test (>= ?gt ?end))
  )
  =>
  (modify ?filter (orders (delete-member$ ?possible-orders ?order-id)))
) 

(defrule production-strategy-select-order
  "select an order for planning from the list of possible orders"
  (order (id ?order-id))
  (production-strategy-order-filter (name possible-orders) (orders $?possible-orders&:(member$ ?order-id ?possible-orders)))
  ?filter <- (production-strategy-order-filter (name selected-orders) (orders $?selected-orders))
  (production-strategy-order-filter (name active-orders) (orders $?active-orders))
  (test (< (+ (length$ ?active-orders) (length$ ?selected-orders)) ?*TOTAL-PRODUCTION-THRESHOLD*))
  (test (not (member$ ?order-id ?selected-orders)))
  =>
  (modify ?filter (orders ?selected-orders ?order-id))
)

(defrule production-strategy-remove-selected-orders
  "remove a order from this filter if
     - it is not possible anymore, or
     - it has been started"
  (order (id ?order-id))
  ?filter <- (production-strategy-order-filter (name selected-orders) (orders $?selected-orders&:(member$ ?order-id ?selected-orders)))
  (production-strategy-order-filter (name possible-orders) (orders $?possible-orders))
  (production-strategy-order-filter (name active-orders) (orders $?active-orders))
  (or
    (test (not (member$ ?order-id ?possible-orders)))
    (test (member$ ?order-id ?active-orders))
  )
  =>
  (modify ?filter (orders (delete-member$ ?selected-orders ?order-id)))
)

(defrule production-strategy-add-active-orders
  ?filter <- (production-strategy-order-filter (name active-orders) (orders $?active-orders))
  (order (id ?order-id) (quantity-requested ?q-req) (quantity-delivered ?q-del&:(<> ?q-del ?q-req)))
  (workpiece-for-order (wp ?wp) (order ?order-id))
  (agenda (plan ?plan-id) (state ACTIVE) (class-selection ?class))
  (pddl-action (id ?action) (plan ?plan-id) (params $?params) (plan-order-class ?p-class&:(>= ?p-class ?class)))
  (test (member$ ?wp ?params))
  (test (not (member$ ?order-id ?active-orders)))
  =>
  (modify ?filter (orders ?active-orders ?order-id))
)

(defrule production-strategy-remove-active-orders
  (order (id ?order-id) (quantity-requested ?q-req) (quantity-delivered ?q-del))
  ?filter <- (production-strategy-order-filter (name active-orders) (orders $?active-orders&:(member$ ?order-id ?active-orders)))
  (or (not (workpiece-for-order (wp ?wp-o) (order ?order-id)))
      (and (workpiece-for-order (wp ?wp) (order ?order-id))
           (test (= ?q-req ?q-del))
      )
  )
  =>
  (modify ?filter (orders (delete-member$ ?active-orders ?order-id)))
)  
