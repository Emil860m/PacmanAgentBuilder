import os
import pickle


class QTableManager:
    def __init__(self, file_name):
        self.directory = "q_tables"
        self.q_table_file_in = file_name + ".pkl"
        self.q_table_file_out = self.get_file_out(file_name) + ".pkl"
        self.q_table = self.load_q_table() if os.path.exists(self.directory) else {}
        print(len(self.q_table.values()))
        self.newTable = {}

    def get_file_out(self, file_name) -> str:
        if "_backup" in file_name:
            return file_name.replace("_backup", "")
        else:
            return file_name + "_backup"


    def getQTable(self):
        return self.q_table

    def getNewTable(self):
        return self.newTable

    def load_q_table(self):
        m = {}
        # for root, dirs, files in os.walk(self.directory):
        #     for name in files:
        with open(self.directory + "/" + self.q_table_file_in, "rb") as f:
            m.update(pickle.load(f))
        return m

    def save_q_table(self):
        self.q_table.update(self.newTable)
        with open(self.directory + "/" + self.q_table_file_out, "wb") as f:
            pickle.dump(self.q_table, f)
