(defglobal
  ?*TOTAL-PRODUCTION-THRESHOLD* = 2
)

(defrule production-strategy-init-filter
  (not (production-strategy-order-list (name possible-orders)))
  =>
  (assert 
    (production-strategy-order-filter (name possible-orders))
    (production-strategy-order-filter (name active-orders))
    (production-strategy-order-filter (name selected-order))
  )
)

(defrule production-strategy-add-possible-order
  "add an order to this filter if it has not been fulfilled and the delivery window is ahead"
  ?filter <- (production-strategy-order-filter (name possible-orders) (orders $?possible-orders))
  (order (id ?order-id) (quantity-requested ?q-req) (quantity-delivered ?q-del&:(< ?q-del ?q-req))
          (delivery-begin ?begin))
  (production-strategy-order-filter (name active-orders) (orders $?active-orders))
  (test (not (member$ ?order-id ?possible-orders)))
  (test (not (member$ ?order-id ?active-orders)))
  (game-time ?gt)
  (test (< ?gt ?begin))
  =>
  (modify ?filter ($?possible-orders ?order-id))
)

(defrule production-strategy-remove-possible-order-active
  "remove an order from this filter if it has been started or fulfilled"
  (order (id ?order-id) (quantity-requested ?q-req) (quantity-delivered ?q-del))
  ?filter <- (production-strategy-order-filter (name possible-orders) (orders $?possible-orders&:(member$ ?order-id ?possible-orders)))
  (or
    (production-strategy-order-filter (name active-orders) (orders $?active-orders&:(member$ ?order-id ?active-orders)))
    (test (= ?q-req ?q-del))
  )
  =>
  (modify ?filter (orders (delete-member$ ?possible-orders ?order-id)))
) 

(defrule production-strategy-select-order
  "select an order for planning from the list of possible orders"
  (order (id ?order-id))
  (production-strategy-order-filter (name possible-orders) (orders $?possible-orders&:(member$ ?order-id ?possible-orders)))
  ?filter <- (production-strategy-order-filter (name selected-orders) (orders $?selected-orders))
  (production-strategy-order-filter (name active-orders) (order $?active-orders))
  (test (< (length$ ?active-orders) ?*TOTAL-PRODUCTION-THRESHOLD*))
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
    (not (member$ ?order-id ?possible-orders))
    (not (member$ ?order-id ?active-orders))
  )
  =>
  (modify ?filter (orders (delete-member$ ?selected-orders ?order-id)))
)

(defrule production-strategy-add-active-orders
  (
