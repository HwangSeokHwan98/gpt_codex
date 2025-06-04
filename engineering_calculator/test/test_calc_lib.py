import math
from engineering_calculator.calc_lib import evaluate_expression


def test_basic_operations():
    assert evaluate_expression('1 + 2 * 3') == 7
    assert evaluate_expression('4 / 2') == 2
    assert evaluate_expression('2 ** 3') == 8


def test_functions():
    assert evaluate_expression('sin(0)') == 0
    assert math.isclose(evaluate_expression('cos(0)'), 1.0)
    assert evaluate_expression('sqrt(4)') == 2


def test_invalid_expression():
    try:
        evaluate_expression('import os')
    except ValueError:
        pass
    else:
        assert False, 'Expected ValueError for invalid expression'
