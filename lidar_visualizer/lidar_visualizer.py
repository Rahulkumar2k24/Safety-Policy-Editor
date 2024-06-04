import sys
import os
import pandas as pd
import sqlite3
from PySide2.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QFileDialog, QVBoxLayout, QWidget, QLabel, QTableWidget, QTableWidgetItem, QHeaderView, QStackedLayout, QSpacerItem, QSizePolicy, QHBoxLayout, QComboBox
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
        
        self.excel_files = {}
        self.file_loaded = False
        
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
        
        # Centered Upload Button with Image
        self.upload_button = QPushButton()
        self.upload_button.setFixedSize(150, 150)
        self.upload_button.setStyleSheet("""
            QPushButton {
                border: none;
                background-color: transparent;
                margin: auto;
            }
            QPushButton:hover {
                background-color: #f0f0f0;
            }
        """)
        self.upload_button.clicked.connect(self.upload_file)
        
        icon = QIcon('/home/fbots/Downloads/iconfinder-upload-4341320_120532.png')  # Path to your image
        self.upload_button.setIcon(icon)
        self.upload_button.setIconSize(self.upload_button.size())
        
        self.home_layout.addWidget(self.upload_button, alignment=Qt.AlignCenter)
        
        self.upload_label = QLabel('Upload Policy File')
        self.upload_label.setAlignment(Qt.AlignCenter)
        self.upload_label.setStyleSheet("font-size: 30px; margin-top: 20px;")
        self.home_layout.addWidget(self.upload_label)
        
        self.info_label = QLabel('Upload a Policy file to update coordinates.')
        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setStyleSheet("font-size: 20px; margin-top: 10px;")
        self.home_layout.addWidget(self.info_label)
        
        # Spacer to push content to the center
        self.home_layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        # Vehicle List Dropdown
        self.vehicle_label = QLabel('Select Vehicle Type:')
        self.vehicle_label.setAlignment(Qt.AlignCenter)
        self.vehicle_label.setStyleSheet("font-size: 20px; margin-top: 15px;")
        self.home_layout.addWidget(self.vehicle_label)

        self.vehicle_dropdown = QComboBox()
        self.vehicle_dropdown.addItems(['BOPT', 'HRSR', 'Fork Lift', 'Reach Truck', 'Accumover'])
        self.vehicle_dropdown.setFixedSize(200, 40)
        self.home_layout.addWidget(self.vehicle_dropdown, alignment=Qt.AlignCenter)
        
        # Zone List Dropdown
        self.zone_label = QLabel('Select Zone:')
        self.zone_label.setAlignment(Qt.AlignCenter)
        self.zone_label.setStyleSheet("font-size: 20px; margin-top: 15px;")
        self.home_layout.addWidget(self.zone_label)

        self.zone_dropdown = QComboBox()
        self.zone_dropdown.addItems(['Fork Side', 'Non Fork Side'])
        self.zone_dropdown.setFixedSize(200, 40)
        self.zone_dropdown.currentIndexChanged.connect(self.update_sub_zone_visibility)
        self.home_layout.addWidget(self.zone_dropdown, alignment=Qt.AlignCenter)
        
        # Sub Zone List Dropdown
        self.sub_zone_label = QLabel('')
        self.sub_zone_label.setAlignment(Qt.AlignCenter)
        self.sub_zone_label.setStyleSheet("font-size: 0px; margin-top: 0px;")
        self.home_layout.addWidget(self.sub_zone_label)

        self.sub_zone_dropdown = QComboBox()
        self.sub_zone_dropdown.addItems(['', ''])
        self.sub_zone_dropdown.setFixedSize(0, 0)
        self.sub_zone_dropdown.currentIndexChanged.connect(self.load_table_after_selection)
        self.home_layout.addWidget(self.sub_zone_dropdown, alignment=Qt.AlignCenter)

        # Initially hide sub-zone components
        self.sub_zone_label.hide()
        self.sub_zone_dropdown.hide()

        # Fork Side Button
        self.fork_side_button = QPushButton('Fork Side')
        self.fork_side_button.setFixedSize(150, 40)
        self.fork_side_button.setStyleSheet("""
            QPushButton {
                background-color: #FF5733;
                color: white;
                font-size: 15px;
                border-radius: 10px;
            }
        """)
        self.fork_side_button.clicked.connect(self.check_and_load_fork_side)
        self.home_layout.addWidget(self.fork_side_button, alignment=Qt.AlignCenter)

        # Sub Zone Buttons (Right and Left)
        self.right_side_button = QPushButton('Right Side')
        self.right_side_button.setFixedSize(150, 40)
        self.right_side_button.setStyleSheet("""
            QPushButton {
                background-color: #FF5733;
                color: white;
                font-size: 15px;
                border-radius: 10px;
            }
        """)
        self.right_side_button.clicked.connect(lambda: self.check_and_load_non_fork_side('Right'))
        self.home_layout.addWidget(self.right_side_button, alignment=Qt.AlignCenter)

        self.left_side_button = QPushButton('Left Side')
        self.left_side_button.setFixedSize(150, 40)
        self.left_side_button.setStyleSheet("""
            QPushButton {
                background-color: #FF5733;
                color: white;
                font-size: 15px;
                border-radius: 10px;
            }
        """)
        self.left_side_button.clicked.connect(lambda: self.check_and_load_non_fork_side('Left'))
        self.home_layout.addWidget(self.left_side_button, alignment=Qt.AlignCenter)

        # Initially hide sub-zone buttons
        self.right_side_button.hide()
        self.left_side_button.hide()

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
        self.table.setColumnCount(6)
        self.table.setHorizontalHeaderLabels(['Field_Set_No', 'Used_For', 'Speed', 'Slow_Safety_Distance_3', 'Stop_Safety_Distance_2', 'Emergency_Distance_1'])
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
        
    def upload_file(self):
        file_name, _ = QFileDialog.getOpenFileName(self, "Open Excel File", "", "Excel Files (*.xlsx *.xls)")
        if file_name:
            zone = self.detect_zone_from_filename(file_name)
            if zone:
                self.excel_files[zone] = file_name
                self.info_label.setText(f'Uploaded {file_name}. Select the zone to create the table.')
            else:
                self.info_label.setText('Invalid file name. Please make sure the file name includes the zone information (e.g., "Fork Side.xlsx").')

    def detect_zone_from_filename(self, file_name):
        if 'fork side' in file_name.lower():
            return 'Fork Side'
        elif 'right side' in file_name.lower():
            return 'Right Side'
        elif 'left side' in file_name.lower():
            return 'Left Side'
        return None

    def read_excel(self, file_name):
        try:
            self.df = pd.read_excel(file_name)
            # Rename columns to avoid conflicts
            self.df.columns = ['Field_Set_No', 'Used_For', 'Slow_Safety_Distance_3', 'Stop_Safety_Distance_2', 'Emergency_Distance_1']
            self.table_info_label.setText(f'Loaded {len(self.df)} coordinates from {file_name}.')
        except Exception as e:
            self.table_info_label.setText(f'Failed to load file: {str(e)}')

    def check_and_load_fork_side(self):
        if 'Fork Side' in self.excel_files:
            self.load_table_after_selection('Fork Side')
        else:
            self.table_info_label.setText('Please upload a valid Fork Side file.')

    def check_and_load_non_fork_side(self, side):
        zone = f'{side} Side'
        if zone in self.excel_files:
            self.load_table_after_selection(zone)
        else:
            self.table_info_label.setText(f'Please upload a valid {zone} file.')

    def load_table_after_selection(self, zone=None):
        if not zone:
            zone = self.zone_dropdown.currentText()
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
            item = QTableWidgetItem(str(row['Field_Set_No']))
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            self.table.setItem(i, 0, item)

            combo_box_used_for = self.create_used_for_combobox(row['Used_For'])
            self.table.setCellWidget(i, 1, combo_box_used_for)

            combo_box_speed = self.create_speed_combobox()
            self.table.setCellWidget(i, 2, combo_box_speed)

            self.table.setItem(i, 3, QTableWidgetItem(str(row['Slow_Safety_Distance_3'])))
            self.table.setItem(i, 4, QTableWidgetItem(str(row['Stop_Safety_Distance_2'])))
            self.table.setItem(i, 5, QTableWidgetItem(str(row['Emergency_Distance_1'])))

    def create_used_for_combobox(self, selected_value=None):
        combo_box = QComboBox()
        combo_box.addItems(['Standing', 'Reverse at speed less than 0.5m/s', 'Reverse at speed less than 1m/s', 'Reverse at speed less than 1.3m/s', 'Forward at speed less than 0.5m/s', 'Forward at speed less than 1m/s', 'Forward at speed less than 1.3m/s', 'Forward with turning left', 'Forward with turning Right', 'Reverse with turn left', 'Reverse with turn right', 'Pickdrop field', 'Parking field', 'Diagnostic field', 'Reverse at speed greater than 1.3m/s', 'Forward at speed greater than 1.3m/s'])
        if selected_value:
            index = combo_box.findText(selected_value, Qt.MatchFixedString)
            if index >= 0:
                combo_box.setCurrentIndex(index)
        return combo_box

    def create_speed_combobox(self):
        combo_box = QComboBox()
        combo_box.addItems(['0m/s', '0.1m/s', '0.2m/s', '0.3m/s', '0.4m/s', '0.5m/s', '0.6m/s', '0.7m/s', '0.8m/s', '0.9m/s', '1m/s', '1.1m/s', '1.2m/s', '1.3m/s', '1.4m/s', '1.5m/s', '1.6m/s', '1.7m/s', '1.8m/s'])
        return combo_box

    def show_table_page(self):
        self.stacked_layout.setCurrentIndex(1)
        
    def show_home_page(self):
        self.stacked_layout.setCurrentIndex(0)

    def add_table_row(self):
        row_position = self.table.rowCount()
        self.table.insertRow(row_position)

        self.table.setItem(row_position, 0, QTableWidgetItem(""))

        combo_box_used_for = self.create_used_for_combobox()
        self.table.setCellWidget(row_position, 1, combo_box_used_for)

        combo_box_speed = self.create_speed_combobox()
        self.table.setCellWidget(row_position, 2, combo_box_speed)

        for column in range(3, self.table.columnCount()):
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
            field_set_no = self.table.item(row, 0).text()
            used_for = self.table.cellWidget(row, 1).currentText()
            speed = self.table.cellWidget(row, 2).currentText()
            slow_distance = self.clean_and_convert_to_float(self.table.item(row, 3).text())
            stop_distance = self.clean_and_convert_to_float(self.table.item(row, 4).text())
            emergency_distance = self.clean_and_convert_to_float(self.table.item(row, 5).text())
            self.coordinates.append((field_set_no, used_for, speed, slow_distance, stop_distance, emergency_distance))
        
        # Determine the SQLite file path based on the selected zone and sub-zone
        selected_zone = self.zone_dropdown.currentText()
        selected_sub_zone = self.sub_zone_dropdown.currentText()
        
        if selected_zone == "Fork Side":
            file_name = '/home/fbots/ldlidar_ros2_ws/lidar_visualizer/DB_Data/Fork_Side.sqlite'
        elif selected_zone == "Non Fork Side":
            if selected_sub_zone == "Right":
                file_name = '/home/fbots/ldlidar_ros2_ws/lidar_visualizer/DB_Data/Right_Side.sqlite'
            elif selected_sub_zone == "Left":
                file_name = '/home/fbots/ldlidar_ros2_ws/lidar_visualizer/DB_Data/Left_Side.sqlite'
            else:
                self.table_info_label.setText(f'Please select a valid Sub-Zone.')
                return
        # else:
        #     self.table_info_label.setText(f'Please select a valid Zone.')
        #     return
        
        try:
            # Connect to SQLite database
            conn = sqlite3.connect(file_name)
            cursor = conn.cursor()
            
            # Create table if it does not exist
            create_table_query = '''
                CREATE TABLE IF NOT EXISTS lidar_data (
                    Field_Set_No TEXT,
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
                INSERT INTO lidar_data (Field_Set_No, Used_For, Speed, Slow_Safety_Distance_3, Stop_Safety_Distance_2, Emergency_Distance_1)
                VALUES (?, ?, ?, ?, ?, ?)
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

    def update_sub_zone_visibility(self):
        selected_zone = self.zone_dropdown.currentText()
        if selected_zone == "Non Fork Side":
            self.sub_zone_label.show()
            self.sub_zone_dropdown.show()
            self.right_side_button.show()
            self.left_side_button.show()
        else:
            self.sub_zone_label.hide()
            self.sub_zone_dropdown.hide()
            self.right_side_button.hide()
            self.left_side_button.hide()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = LidarVisualizerUI()
    window.show()
    sys.exit(app.exec_())
