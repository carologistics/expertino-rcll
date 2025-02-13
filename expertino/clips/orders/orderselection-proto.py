import concurrent.futures
import time

MAX_TIMESTEPS = 20  # Maximum allowed timesteps for the entire process

class Order:
    def __init__(self, order_id, arrival_time, process_time, window_start, window_end):
        self.order_id = order_id
        self.arrival_time = arrival_time  # Time the order arrives from the refbox
        self.process_time = process_time
        self.window_start = window_start  # Earliest allowed start time
        self.window_end = window_end      # Max finish time

class SelectedOrder:
    def __init__(self, order_id, start_time, finish_time, punctuality):
        self.order_id = order_id 
        self.start_time = start_time
        self.finish_time = finish_time
        self.punctuality = punctuality  # "on-time" or "tardy"

    def __str__(self):
        return f"Order ID: {self.order_id}, Start Time: {self.start_time}, Finish Time: {self.finish_time}, {self.punctuality}"

def evaluate_order(order, current_time):
    window_length = order.window_end - order.window_start
    max_finish = order.window_end + window_length

    # Earliest possible start time is the maximum of current time, arrival time, and window start
    earliest_start = max(current_time, order.arrival_time)
    
    # If earliest start plus process time exceeds MAX_TIMESTEPS, order can't be completed
    if earliest_start + order.process_time > MAX_TIMESTEPS:
        return None

    finish_time = earliest_start + order.process_time

    if finish_time <= order.window_start:
        return None
    elif finish_time <= order.window_end:
        punctuality = "on-time"
    elif finish_time <= max_finish:
        punctuality = "tardy"
    else:
        return None  # Order would finish too late

    return SelectedOrder(order.order_id, earliest_start, finish_time, punctuality)

def process_order(selected_order):
    print(f"Processing Order {selected_order.order_id} starting at time {selected_order.start_time}...")
    time.sleep(selected_order.finish_time - selected_order.start_time)  # Simulate processing time
    print(f"Order {selected_order.order_id} completed at time {selected_order.finish_time}.")
    return selected_order

def main():
    orders = [
        Order(order_id=1, arrival_time=2, process_time=4, window_start=8, window_end=12),
        Order(order_id=2, arrival_time=1, process_time=4, window_start=17, window_end=20),
        Order(order_id=3, arrival_time=4, process_time=9, window_start=8, window_end=12),
        Order(order_id=4, arrival_time=12, process_time=6, window_start=15, window_end=18)
    ]

    selected_orders = []
    current_time = 0

    while current_time < MAX_TIMESTEPS and orders:
        selected_order = None
        for order in orders[:]:  # Iterate over a copy of the list
            if order.arrival_time <= current_time: 
                result = evaluate_order(order, current_time)
                if result:
                    selected_order = result
                    orders.remove(order)
                    break  # Select this order for processing
        
        if selected_order:
            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                future = executor.submit(process_order, selected_order)
                completed_order = future.result()
                selected_orders.append(completed_order)

        
        current_time += 1  # Increment time if no order was selected

    print("\nAll order processes completed or time limit reached.\n")
    if selected_orders:
        print("Final Selected and Processed Orders:")
        for so in selected_orders:
            print(so)
    else:
        print("No orders were selected and processed.")

if __name__ == "__main__":
    main()
