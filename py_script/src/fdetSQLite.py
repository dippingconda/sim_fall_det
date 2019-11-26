import os
import sqlite3
from sqlite3 import Error


class FallDetSQLite:

    def __init__(self, path, db_name, table_name):
        self.path = path
        self.db_name = db_name
        self.table_name = table_name
        self.conn = sqlite3.connect(path + '/' + db_name + '.db')
        self.cursor = self.conn.cursor()

    def create_table(self):
        try:
            sql = '''CREATE TABLE '{table}'(id text, time text, ''' \
                  '''angular_vel_x real, angular_vel_y real, angular_vel_z real, ''' \
                  '''linear_acc_x real, linear_acc_y real, linear_acc_z real) '''.format(table=self.table_name)
            self.cursor.execute(sql)
            self.conn.commit()
        except Error as e:
            print(e)
        finally:
            if self.conn:
                self.conn.close()

    def create_connection(self, path, db_name):
        self.path = path
        self.db_name = db_name
        try:
            self.conn = sqlite3.connect(self.path + '/' + self.db_name + '.db')
            self.cursor = self.conn.cursor()
        except Error as e:
            print(e)
#        finally:
#            return self.conn

    def close_connection(self):
        try:
            if self.conn:
                self.conn.commit()
                self.conn.close()
        except Error as e:
            print(e)
        finally:
            print("DB has benn closed")



    def save_data(self, data):
        sql = '''INSERT INTO '{table}'('id', 'time', 'angular_vel_x', 'angular_vel_y', 'angular_vel_z', 'linear_acc_x', 'linear_acc_y', 'linear_acc_z') VALUES (?, ?, ?, ?, ?, ?, ?, ?)'''.format(table=self.table_name)
        try:
            self.cursor.execute(sql, data)
            self.conn.commit()
        except Error as e:
            print(e)

    def update_data(self, data):
        sql = '''UPDATE '{table}'('id', 'time', 'angular_vel_x', 'angular_vel_y', 'angular_vel_z', 'linear_acc_x', 'linear_acc_y', ''' \
              ''''linear_acc_z') VALUES (?, ?, ?, ?, ?, ?, ?, ?) '''.format(table=self.table_name)
        try:
            self.cursor.execute(sql, data)
            self.conn.commit()
        except Error as e:
            print(e)

    def delete_data_by_id(self, num_id):
        sql = "DELETE FROM {table} WHERE id=?".format(table=self.table_name)
        try:
            self.cursor.execute(sql, (num_id,))
            self.conn.commit()
        except Error as e:
            print(e)

    def delete_data_by_time(self, num_time):
        sql = "DELETE FROM '{table}' WHERE time=?".format(table=self.table_name)
        try:
            self.cursor.execute(sql, (num_time,))
            self.conn.commit()
        except Error as e:
            print(e)

    def delete_data_by_id_time(self, num_id, num_time):
        sql = "DELETE FROM {table} WHERE id = ? AND time = ?".format(table=self.table_name)
        try:
            self.cursor.execute(sql, (num_id, num_time ))
            self.conn.commit()
        except Error as e:
            print(e)

    def delete_data_all(self):
        sql = "DELETE FROM '{table}'".format(table=self.table_name)
        try:
            self.cursor.execute(sql)
            self.conn.commit()
        except Error as e:
            print(e)

    def fetch_all_imu_data(self):
        query = "SELECT * FROM {table}".format(table=self.table_name)
        try:
            self.cursor.execute(query)
            rows = self.cursor.fetchall()
            for row in rows:
                print(row)
            return rows
        except Error as e:
            print(e)

    def fetch_imu_data_by_time(self, query_time):
        sql = "SELECT * FROM '{table}' WHERE time=?".format(table=self.table_name)
        try:
            self.cursor.execute(sql, (query_time,))
            rows = self.cursor.fetchall()
            for row in rows:
                print(row)
            return rows
        except Error as e:
            print(e)


def main():
    home_dir = os.path.expanduser('~')
    imu_data_dir = '/imu_log'
    sqlite_handler = FallDetSQLite(path=home_dir+imu_data_dir, db_name='imu_DB', table_name='imu_data')
    sqlite_handler.create_table()
    sqlite_handler.create_connection(path=home_dir+imu_data_dir, db_name='imu_DB')

    data = ['1', '1', '2', '3', '4', '5', '6', '7']
    sqlite_handler.save_data(data)
    data = ['1', '2', '2', '3', '4', '5', '6', '7']
    sqlite_handler.save_data(data)
    data = ['2', '2', '2', '3', '4', '5', '6', '7']
    sqlite_handler.save_data(data)
    print(sqlite_handler.path)
#    sqlite_handler.fetch_all_imu_data()
#    sqlite_handler.fetch_imu_data_by_time(query_time=2)
#    sqlite_handler.delete_data_by_time(num_time='11')
#    sqlite_handler.delete_data_by_id(num_id=10)
    sqlite_handler.delete_data_all()
    sqlite_handler.close_connection()


if __name__ == '__main__':
    main()