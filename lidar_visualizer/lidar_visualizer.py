import sys
import os
import pandas as pd
import sqlite3
from PySide2.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel, QTableWidget, QTableWidgetItem, QHeaderView, QStackedLayout, QSpacerItem, QSizePolicy, QHBoxLayout, QComboBox, QMessageBox
)
from PySide2.QtCore import Qt
from PySide2.QtGui import QIcon, QPixmap, QFont

class LidarVisualizerUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Safety Policy Editor')
        self.setGeometry(100, 100, 1000, 600)
        
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        
        self.stacked_layout = QStackedLayout(self.central_widget)
        
        self.init_home_page()
        self.init_table_page()
        
        self.stacked_layout.setCurrentIndex(0)
        
        self.excel_files = {
            'Fork Side': '/home/fbots/ldlidar_ros2_ws/Lidar_Data/Lidar_Data_Fork side.xlsx',
            'Left Side': '/home/fbots/ldlidar_ros2_ws/Lidar_Data/Lidar_Data_Left side.xlsx',
            'Right Side': '/home/fbots/ldlidar_ros2_ws/Lidar_Data/Lidar_Data_Right side.xlsx'
        }

        self.speed_values = ['0m/s', '0.1m/s', '0.2m/s', '0.3m/s', '0.4m/s', '0.5m/s', '0.6m/s', '0.7m/s', '0.8m/s', '0.9m/s', '1m/s', '1.1m/s', '1.2m/s', '1.3m/s', '1.4m/s', '1.5m/s', '1.6m/s']

    def init_home_page(self):
        self.home_page = QWidget()
        self.home_layout = QVBoxLayout(self.home_page)
        
        # Logo and header
        self.logo_layout = QHBoxLayout()
        self.logo_label = QLabel()
        logo_path = '/home/fbots/Downloads/1663754824671-removebg-preview(1).png'  # Path to your logo
        pixmap = QPixmap(logo_path)
        self.logo_label.setPixmap(pixmap.scaled(170, 170, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.logo_layout.addWidget(self.logo_label)
        
        self.header_label = QLabel('Safety Policy Editor')
        self.header_label.setFont(QFont('Arial', 24, QFont.Bold))
        self.header_label.setAlignment(Qt.AlignCenter)
        
        self.home_layout.addLayout(self.logo_layout)
        self.home_layout.addWidget(self.header_label)
        
        # Spacer to push content to the center
        self.home_layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        
        # Vehicle Type Dropdown
        self.vehicle_label = QLabel('Select Vehicle Type:')
        self.vehicle_label.setAlignment(Qt.AlignCenter)
        self.vehicle_label.setStyleSheet("font-size: 20px; margin-top: 15px;")
        self.home_layout.addWidget(self.vehicle_label)

        self.vehicle_dropdown = QComboBox()
        self.vehicle_dropdown.addItems(['BOPT', 'HRSR', 'Fork Lift', 'Reach Truck', 'Accumover'])
        self.vehicle_dropdown.setFixedSize(200, 40)
        self.home_layout.addWidget(self.vehicle_dropdown, alignment=Qt.AlignCenter)
        
        # Zone Dropdown
        self.zone_label = QLabel('Select Zone:')
        self.zone_label.setAlignment(Qt.AlignCenter)
        self.zone_label.setStyleSheet("font-size: 20px; margin-top: 15px;")
        self.home_layout.addWidget(self.zone_label)

        self.zone_dropdown = QComboBox()
        self.zone_dropdown.addItems(['Fork Side', 'Non Fork Side'])
        self.zone_dropdown.setFixedSize(200, 40)
        self.zone_dropdown.currentIndexChanged.connect(self.update_sub_zone_visibility)
        self.home_layout.addWidget(self.zone_dropdown, alignment=Qt.AlignCenter)
        
        # Sub Zone Dropdown
        self.sub_zone_label = QLabel('Select Sub Zone:')
        self.sub_zone_label.setAlignment(Qt.AlignCenter)
        self.sub_zone_label.setStyleSheet("font-size: 20px; margin-top: 15px;")
        self.home_layout.addWidget(self.sub_zone_label)

        self.sub_zone_dropdown = QComboBox()
        self.sub_zone_dropdown.addItems([''])
        self.sub_zone_dropdown.setFixedSize(200, 40)
        self.home_layout.addWidget(self.sub_zone_dropdown, alignment=Qt.AlignCenter)
        
        # Initially hide sub-zone components
        self.sub_zone_label.hide()
        self.sub_zone_dropdown.hide()

        # Create Policy File Button
        self.create_policy_button = QPushButton('Create Policy File')
        self.create_policy_button.setFixedSize(200, 60)
        self.create_policy_button.setStyleSheet("""
            QPushButton {
                background-color: #FF5733;
                color: white;
                font-size: 20px;
                font-weight: bold;
                border-radius: 10px;
            }
        """)
        self.create_policy_button.clicked.connect(self.create_policy_file)
        self.home_layout.addWidget(self.create_policy_button, alignment=Qt.AlignCenter)

        # Spacer to push content to the center
        self.home_layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        
        self.stacked_layout.addWidget(self.home_page)
        
    def init_table_page(self):
        self.table_page = QWidget()
        self.table_layout = QVBoxLayout(self.table_page)
        
        self.table_info_label = QLabel('Data from uploaded Excel file:')
        self.table_info_label.setAlignment(Qt.AlignCenter)
        self.table_layout.addWidget(self.table_info_label)
        
        # Table to display coordinates
        self.table = QTableWidget()
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels(['Used_For', 'Speed', 'Slow_Safety_Distance_3', 'Stop_Safety_Distance_2', 'Emergency_Distance_1'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_layout.addWidget(self.table)

        # Buttons for adding, deleting rows, saving and back
        self.button_layout = QHBoxLayout()

        self.add_row_button = QPushButton('Add Row')
        self.add_row_button.setFixedSize(160, 60)  # Compact size
        self.add_row_button.setStyleSheet("""
            QPushButton {
                background-color: #FF5733;
                color: white;
                font-size: 25px;
                font-weight: bold;
                border-radius: 20px;
            }
        """)
        self.add_row_button.clicked.connect(self.add_table_row)
        self.button_layout.addWidget(self.add_row_button)

        self.delete_row_button = QPushButton('Delete Row')
        self.delete_row_button.setFixedSize(160, 60)  # Compact size
        self.delete_row_button.setStyleSheet("""
            QPushButton {
                background-color: #FF5733;
                color: white;
                font-size: 25px;
                font-weight: bold;
                border-radius: 20px;
            }
        """)
        self.delete_row_button.clicked.connect(self.delete_table_row)
        self.button_layout.addWidget(self.delete_row_button)

        self.save_button = QPushButton('Save to SQLite')
        self.save_button.setFixedSize(160, 60)  # Compact size
        self.save_button.setStyleSheet("""
            QPushButton {
                background-color: #FF5733;
                color: white;
                font-size: 20px;
                font-weight: bold;
                border-radius: 20px;
            }
        """)
        self.save_button.setIcon(QIcon.fromTheme("document-save"))
        self.save_button.clicked.connect(self.save_to_db)
        self.button_layout.addWidget(self.save_button)
        
        self.back_button = QPushButton('Back')
        self.back_button.setFixedSize(160, 60)  # Compact size
        self.back_button.setStyleSheet("""
            QPushButton {
                background-color: #FF5733;
                color: white;
                font-size: 25px;
                font-weight: bold;
                border-radius: 20px;
            }
        """)
        self.back_button.clicked.connect(self.show_home_page)
        self.button_layout.addWidget(self.back_button)

        self.table_layout.addLayout(self.button_layout)
        
        self.stacked_layout.addWidget(self.table_page)
    
    def update_sub_zone_visibility(self):
        selected_zone = self.zone_dropdown.currentText()
        print(f"Debug: selected_zone in update_sub_zone_visibility = {selected_zone}")
        if selected_zone == "Non Fork Side":
            self.sub_zone_label.show()
            self.sub_zone_dropdown.clear()
            self.sub_zone_dropdown.addItems(['Right Side', 'Left Side'])
            self.sub_zone_dropdown.show()
        else:
            self.sub_zone_label.hide()
            self.sub_zone_dropdown.hide()

    def create_policy_file(self):
        vehicle_type = self.vehicle_dropdown.currentText()
        selected_zone = self.zone_dropdown.currentText()
        selected_sub_zone = self.sub_zone_dropdown.currentText() if self.sub_zone_dropdown.isVisible() else None
        
        print(f"Debug: vehicle_type = {vehicle_type}, selected_zone = {selected_zone}, selected_sub_zone = {selected_sub_zone}")
        
        if vehicle_type != 'BOPT':
            QMessageBox.critical(self, 'Error', 'Only BOPT vehicle type is supported.')
            return
        
        if selected_zone == 'Fork Side':
            self.load_table_after_selection('Fork Side')
        elif selected_zone == 'Non Fork Side':
            if selected_sub_zone == 'Right Side':
                self.load_table_after_selection('Right Side')
            elif selected_sub_zone == 'Left Side':
                self.load_table_after_selection('Left Side')
            else:
                QMessageBox.critical(self, 'Error', 'Please select a valid sub zone.')
                return
        else:
            QMessageBox.critical(self, 'Error', 'Please select a valid zone.')

    def read_excel(self, file_name):
        try:
            self.df = pd.read_excel(file_name)
            # Rename columns to avoid conflicts
            self.df.columns = ['Field_Set_No', 'Used_For', 'Speed', 'Slow_Safety_Distance_3', 'Stop_Safety_Distance_2', 'Emergency_Distance_1']
            # Extract the unique values for Used_For and Speed
            self.used_for_values = self.df['Used_For'].unique().tolist()
            # Combine unique speed values from the file with the predefined speed values
            self.speed_values = list(set(self.speed_values + self.df['Speed'].unique().tolist()))
            self.df = self.df.drop(columns=['Field_Set_No'])  # Drop the Field_Set_No column
            self.table_info_label.setText(f'Loaded {len(self.df)} coordinates from {file_name}.')
        except KeyError as e:
            self.table_info_label.setText(f'Key error: {e}')
        except Exception as e:
            self.table_info_label.setText(f'Failed to load file: {str(e)}')

    def load_table_after_selection(self, zone):
        print(f"Debug: Loading table for zone: {zone}")
        if zone in self.excel_files:
            self.read_excel(self.excel_files[zone])
            self.update_table()
            self.show_table_page()
        else:
            self.table_info_label.setText(f'Please upload a valid {zone} file.')

    def update_table(self):
        self.table.setRowCount(0)
        for i, row in self.df.iterrows():
            self.table.insertRow(i)

            combo_box_used_for = self.create_combobox(row['Used_For'], self.used_for_values)
            self.table.setCellWidget(i, 0, combo_box_used_for)

            combo_box_speed = self.create_combobox(row['Speed'], self.speed_values)
            self.table.setCellWidget(i, 1, combo_box_speed)

            self.table.setItem(i, 2, QTableWidgetItem(str(row['Slow_Safety_Distance_3'])))
            self.table.setItem(i, 3, QTableWidgetItem(str(row['Stop_Safety_Distance_2'])))
            self.table.setItem(i, 4, QTableWidgetItem(str(row['Emergency_Distance_1'])))

    def create_combobox(self, selected_value, options):
        combo_box = QComboBox()
        combo_box.addItems(options)
        if selected_value:
            index = combo_box.findText(selected_value, Qt.MatchFixedString)
            if index >= 0:
                combo_box.setCurrentIndex(index)
        return combo_box

    def show_table_page(self):
        self.stacked_layout.setCurrentIndex(1)
        
    def show_home_page(self):
        self.stacked_layout.setCurrentIndex(0)

    def add_table_row(self):
        row_position = self.table.rowCount()
        self.table.insertRow(row_position)

        combo_box_used_for = self.create_combobox('', self.used_for_values)
        self.table.setCellWidget(row_position, 0, combo_box_used_for)

        combo_box_speed = self.create_combobox('', self.speed_values)
        self.table.setCellWidget(row_position, 1, combo_box_speed)

        for column in range(2, self.table.columnCount()):
            self.table.setItem(row_position, column, QTableWidgetItem(""))

    def delete_table_row(self):
        current_row = self.table.currentRow()
        if current_row >= 0:
            self.table.removeRow(current_row)

    def save_to_db(self):
        row_count = self.table.rowCount()
        if row_count == 0:
            self.table_info_label.setText('No data to save. Please upload an Excel file first.')
            return

        self.coordinates = []
        for row in range(row_count):
            used_for = self.table.cellWidget(row, 0).currentText()
            speed = self.table.cellWidget(row, 1).currentText()
            slow_distance = self.clean_and_convert_to_float(self.table.item(row, 2).text())
            stop_distance = self.clean_and_convert_to_float(self.table.item(row, 3).text())
            emergency_distance = self.clean_and_convert_to_float(self.table.item(row, 4).text())
            self.coordinates.append((used_for, speed, slow_distance, stop_distance, emergency_distance))
        
        # Determine the SQLite file path based on the selected zone
        selected_zone = self.zone_dropdown.currentText()
        selected_sub_zone = self.sub_zone_dropdown.currentText() 

        print(f"Debug: selected_zone = {selected_zone}, selected_sub_zone = {selected_sub_zone}")
        
        if selected_zone == "Fork Side":
            file_name = '/home/fbots/ldlidar_ros2_ws/lidar_visualizer/DB_Data/Fork_Side.sqlite'
        elif selected_zone == "Non Fork Side":
            if selected_sub_zone == "Right Side":
                file_name = '/home/fbots/ldlidar_ros2_ws/lidar_visualizer/DB_Data/Right_Side.sqlite'
            elif selected_sub_zone == "Left Side":
                file_name = '/home/fbots/ldlidar_ros2_ws/lidar_visualizer/DB_Data/Left_Side.sqlite'
            else:
                self.table_info_label.setText(f'Please select a valid Sub-Zone.')
                return
        else:
            self.table_info_label.setText(f'Please select a valid Zone.')
            return
        
        print(f"Debug: Saving to {file_name}")
        
        try:
            # Connect to SQLite database
            conn = sqlite3.connect(file_name)
            cursor = conn.cursor()
            
            # Create table if it does not exist
            create_table_query = '''
                CREATE TABLE IF NOT EXISTS lidar_data (
                    Used_For TEXT,
                    Speed TEXT,
                    Slow_Safety_Distance_3 REAL,
                    Stop_Safety_Distance_2 REAL,
                    Emergency_Distance_1 REAL
                )
            '''
            cursor.execute(create_table_query)
            
            # Clear existing data
            cursor.execute('DELETE FROM lidar_data')
            
            # Insert data
            insert_query = '''
                INSERT INTO lidar_data (Used_For, Speed, Slow_Safety_Distance_3, Stop_Safety_Distance_2, Emergency_Distance_1)
                VALUES (?, ?, ?, ?, ?)
            '''
            cursor.executemany(insert_query, self.coordinates)
            
            # Commit and close
            conn.commit()
            conn.close()
            
            self.table_info_label.setText(f'Successfully saved to {file_name}.')
        except Exception as e:
            self.table_info_label.setText(f'Failed to save file: {str(e)}')
            print(f"Error: {str(e)}")

    def clean_and_convert_to_float(self, value):
        try:
            # Remove any non-numeric characters except for the decimal point
            cleaned_value = ''.join(char for char in value if char.isdigit() or char == '.')
            return float(cleaned_value)
        except ValueError:
            return 0.0  # Return a default value if conversion fails

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = LidarVisualizerUI()
    window.show()
    sys.exit(app.exec_())
