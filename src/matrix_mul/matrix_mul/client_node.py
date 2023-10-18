import sys
import numpy as np
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import MatrixMul

class MultiplicationClientAsync(Node):
    def __init__(self):
        super().__init__("multiplication_client_async")
        self.client_ = self.create_client(MatrixMul, "matrix_mul")
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server MatrixMul...")

    
    def send_request(self):
        a = np.loadtxt("a.txt")
        b = np.loadtxt("b.txt")
        a_shape = a.shape
        b_shape = b.shape
        print(a_shape)
        print(b_shape)  
        request = MatrixMul.Request()
        request.a = a.flatten().tolist()  # Flatten and convert to list
        request.b = b.flatten().tolist()  # Flatten and convert to list
        request.a_shape = a_shape.tolist()  # Convert to list
        request.b_shape = b_shape.tolist()  # Convert to list

        self.future = self.client_.call_async(request)
       


def main(args=None):
    rclpy.init(args=args)
    #create node
    multiplication_client = MultiplicationClientAsync()
    #send request
    multiplication_client.send_request()
    #spin node
    while rclpy.ok():
        rclpy.spin_once(multiplication_client)
        if multiplication_client.future.done():
            try:
                response = multiplication_client.future.result()
            except Exception as e:
                multiplication_client.get_logger().error("Service call failed {e}")
            else:
                response = multiplication_client.future.result()
                result = np.array(response.product).reshape(response.product_shape)
                multiplication_client.get_logger().info("Result of multiplication:\n%s" % result)
                # multiplication_client.get_logger().info("Result of multiplication: %d" % response.product)
            break
    #destroy node and shutdown
    multiplication_client.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()