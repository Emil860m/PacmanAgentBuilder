import os
import pickle


class QTableManager:
    def __init__(self, file_name):
        self.directory = "q_tables"
        self.q_table_file = file_name
        self.q_table = self.load_q_table() if os.path.exists(self.directory) else {}
        self.newTable = {}

    def getQTable(self):
        return self.q_table

    def getNewTable(self):
        return self.newTable

    def load_q_table(self):
        m = {}
        for root, dirs, files in os.walk(self.directory):
            for name in files:
                with open(self.directory + "/" + name, "rb") as f:
                    m.update(pickle.load(f))
        return m

    def save_q_table(self):
        with open(self.directory + "/" + self.q_table_file, "wb") as f:
            pickle.dump(self.newTable, f)
