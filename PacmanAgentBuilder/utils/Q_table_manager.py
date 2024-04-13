import os
import pickle


class QTableManager:
    def __init__(self):
        self.q_table_file = "q_table.pkl"
        self.q_table = self.load_q_table() if os.path.exists(self.q_table_file) else {}

    def getQTable(self):
        return self.q_table

    def load_q_table(self):
        with open(self.q_table_file, "rb") as f:
            return pickle.load(f)

    def save_q_table(self):
        with open(self.q_table_file, "wb") as f:
            pickle.dump(self.q_table, f)
