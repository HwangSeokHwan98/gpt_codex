import ast
import operator
import math

# Allowed binary operators mapping from ast to functions
_ALLOWED_OPERATORS = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
    ast.Pow: operator.pow,
    ast.Mod: operator.mod,
}

_ALLOWED_UNARY_OPERATORS = {
    ast.UAdd: operator.pos,
    ast.USub: operator.neg,
}

_ALLOWED_FUNCTIONS = {
    'sin': math.sin,
    'cos': math.cos,
    'tan': math.tan,
    'sqrt': math.sqrt,
    'log': math.log,
    'exp': math.exp,
}


class SafeEval(ast.NodeVisitor):
    def visit(self, node):
        if isinstance(node, ast.Expression):
            return self.visit(node.body)
        elif isinstance(node, ast.Num):
            return node.n
        elif isinstance(node, ast.BinOp):
            op_type = type(node.op)
            if op_type not in _ALLOWED_OPERATORS:
                raise ValueError(f"Operator {op_type} not allowed")
            left = self.visit(node.left)
            right = self.visit(node.right)
            return _ALLOWED_OPERATORS[op_type](left, right)
        elif isinstance(node, ast.UnaryOp):
            op_type = type(node.op)
            if op_type not in _ALLOWED_UNARY_OPERATORS:
                raise ValueError(f"Unary operator {op_type} not allowed")
            operand = self.visit(node.operand)
            return _ALLOWED_UNARY_OPERATORS[op_type](operand)
        elif isinstance(node, ast.Call):
            if not isinstance(node.func, ast.Name):
                raise ValueError("Only simple function calls allowed")
            func_name = node.func.id
            if func_name not in _ALLOWED_FUNCTIONS:
                raise ValueError(f"Function {func_name} not allowed")
            args = [self.visit(arg) for arg in node.args]
            return _ALLOWED_FUNCTIONS[func_name](*args)
        else:
            raise ValueError(f"Unsupported expression: {ast.dump(node)}")


def evaluate_expression(expression: str) -> float:
    """Safely evaluate a mathematical expression."""
    try:
        parsed = ast.parse(expression, mode='eval')
        evaluator = SafeEval()
        return evaluator.visit(parsed)
    except Exception as exc:
        raise ValueError(str(exc))
