# message_bus.py

import threading
from collections import defaultdict

class MessageBus:
    def __init__(self):
        # Dictionary mapping topic → list of callback functions (subscribers)
        self.subscribers = defaultdict(list)
        self.lock = threading.Lock()

    def subscribe(self, topic, callback):
        """Register a function to receive messages on a specific topic."""
        with self.lock:
            self.subscribers[topic].append(callback)

    def publish(self, topic, message):
        """Send a message to all subscribers of the given topic."""
        with self.lock:
            callbacks = list(self.subscribers[topic])  # make a copy

        # Call outside the lock to avoid blocking other operations
        for callback in callbacks:
            callback(topic, message)
