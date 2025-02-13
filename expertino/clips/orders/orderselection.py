class Order:
    def __init__(self, order_id, start_time, process_time, window_start, window_end):
        self.order_id = order_id
        self.start_time = start_time
        self.process_time = process_time
        self.window_start = window_start    # Earliest allowed finish time (delivery station availability)
        self.window_end = window_end        # Ideal finish time (deadline)
    
    @property
    def finish_time(self):
        return self.start_time + self.process_time

class SelectedOrder:
    def __init__(self, order_id, finish_time, punctuality):
        self.order_id = order_id
        self.finish_time = finish_time
        self.punctuality = punctuality    # either "on-time" or "tardy"

    def __str__(self):
        return f"Order ID: {self.order_id}, Finish Time: {self.finish_time}, {self.punctuality}"

def evaluate_order(order):
    finish = order.finish_time
    # Calculate the length of the delivery window
    window_length = order.window_end - order.window_start
    # Maximum allowed finish time equals the deadline plus the window length.
    max_finish = order.window_end + window_length

    if finish < order.window_start:
        print(f"Order {order.order_id} not selected: completion too early "
              f"(finish = {finish} is before window-start = {order.window_start}).")
        return None
    elif finish <= order.window_end:
        print(f"Order {order.order_id} selected as on-time "
              f"(finish = {finish} is within the window [{order.window_start}, {order.window_end}]).")
        return SelectedOrder(order.order_id, finish, "on-time")
    elif finish <= max_finish:
        print(f"Order {order.order_id} selected as tardy "
              f"(finish = {finish} is after the window but before the max allowed finish = {max_finish}).")
        return SelectedOrder(order.order_id, finish, "tardy")
    else:
        print(f"Order {order.order_id} not selected: completion too late "
              f"(finish = {finish} is after the allowed maximum {max_finish}).")
        return None

def main():
    # Create three dummy orders
    orders = [
        Order(order_id=1, start_time=2, process_time=4, window_start=8, window_end=12),  # finishes at 6 (too early)
        Order(order_id=2, start_time=5, process_time=4, window_start=8, window_end=12),  # finishes at 9 (on-time)
        Order(order_id=3, start_time=4, process_time=9, window_start=8, window_end=12)   # finishes at 13 (tardy if within allowed)
    ]

    selected_orders = []

    # Evaluate all orders based on their finish times
    for order in orders:
        result = evaluate_order(order)
        if result:
            selected_orders.append(result)

    # Display all selected orders
    if selected_orders:
        print("\nFinal Selected Orders:")
        for s_order in selected_orders:
            print(s_order)
    else:
        print("\nNo orders were selected.")

if __name__ == "__main__":
    main()
