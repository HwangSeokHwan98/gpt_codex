import rclpy
from rclpy.node import Node

from engineering_calculator.srv import CalculateExpression
from .calc_lib import evaluate_expression


class CalculatorService(Node):
    def __init__(self):
        super().__init__('calculator_service')
        self.srv = self.create_service(
            CalculateExpression,
            'calculate_expression',
            self.calculate_callback)

    def calculate_callback(self, request, response):
        try:
            result = evaluate_expression(request.expression)
            response.result = float(result)
            response.error = ''
        except Exception as exc:
            response.result = float('nan')
            response.error = str(exc)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CalculatorService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
