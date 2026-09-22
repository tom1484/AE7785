- What is a ROS\_DOMAIN\_ID? (2pt)

    ```text
    An integer that separates ROS 2 communication networks. Nodes with different ROS_DOMAIN_IDs normally cannot discover or communicate with each other.
    ```

- What is a node? (2pt)

    ```text
    A ROS 2 process/component that performs computation and communicates with other nodes through topics, services, actions, etc.
    ```

- What is a topic? (2pt)

    ```text
    A named channel used for asynchronous data exchange between publishers and subscribers.
    ```

- What is a message? (2pt)

    ```text
    A structured data type sent over a topic, such as Int64, Float64, or something more complex like Twist.
    ```

- What is a subscriber? Write the syntax to create a subscriber that subscribes to the topic amazing\_int, which takes message of type UInt64, and uses the callback function magic\_fun, in C++ or Python. Note: This should only be a few lines of code, not a full script. (5pt)

    ```text
    Receives messages from a topic and calls a callback when a new message arrives.
    ```

    ```python
    from std_msgs.msg import Bool
    
    # Inside a node
    subscription = self.create_subscription(
        UInt64, 'amazing_int', self.amazing_callback, 10)
    ```

- What is a publisher? Write the syntax to create a publisher that publishes to the topic amazing\_bool, which takes message of type Bool, in Python. Note: This should only be a few lines of code, not a full script. (5pt)

    ```text
    Sends messages to a topic.
    ```

    ```python
    from std_msgs.msg import Bool

    # Inside a node
    publisher = self.create_publisher(
        Bool, 'amazing_bool', 10)
    ```

- Can a node have multiple subscribers? Can a node have multiple publishers? (2pt)

    ```text
    Yes. A single node can have multiple subscribers and multiple publishers, including multiple ones for different topics.
    ```
