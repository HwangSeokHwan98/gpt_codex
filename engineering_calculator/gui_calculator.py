import tkinter as tk
from tkinter import ttk, messagebox

from engineering_calculator.calc_lib import evaluate_expression


class CalculatorUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Engineering Calculator")
        self._create_widgets()

    def _create_widgets(self):
        self.expression_entry = ttk.Entry(self, width=40)
        self.expression_entry.grid(row=0, column=0, padx=5, pady=5, columnspan=2)
        self.expression_entry.focus()

        calc_button = ttk.Button(self, text="Calculate", command=self._calculate)
        calc_button.grid(row=1, column=0, padx=5, pady=5)

        self.result_var = tk.StringVar(value="0")
        result_label = ttk.Label(self, textvariable=self.result_var)
        result_label.grid(row=1, column=1, padx=5, pady=5)

    def _calculate(self):
        expr = self.expression_entry.get()
        try:
            result = evaluate_expression(expr)
            self.result_var.set(str(result))
        except Exception as exc:
            messagebox.showerror("Error", str(exc))


def main():
    app = CalculatorUI()
    app.mainloop()


if __name__ == "__main__":
    main()
