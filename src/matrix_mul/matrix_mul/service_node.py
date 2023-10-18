import numpy as np
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import MatrixMul

class MultiplicationService(Node):
    def __init__(self):
        super().__init__("multiplication_service")
        self.service_ = self.create_service(MatrixMul, "matrix_mul", self.matrix_mul_callback)
    
    def matrix_mul_callback(self, request, response):
        try:
            a = np.array(request.a).reshape(request.a_shape)  # Reshape 'a'
            b = np.array(request.b).reshape(request.b_shape)  # Reshape 'b'
            result = np.dot(a, b)
            print(result)
            response.product = result.flatten().tolist()  # Flatten the result and store in response
            # Get the product shape from the result of np.dot
            response.product_shape = result.shape
            response.error_message = "Success"
        except ValueError as e:
            response.error_message = str(e)
            response.product = []  # Clear the product
            response.product_shape = [0, 0]  # Set a shape for the empty product
            self.get_logger().error(f"Matrix multiplication error: {str(e)}")  # Log the error messages
        return response
    

def main(args=None):
    rclpy.init(args=args)
    #create node
    multiplication_service = MultiplicationService()
    #spin node
    rclpy.spin(multiplication_service)
    #destroy node and shutdown
    multiplication_service.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
