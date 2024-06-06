# Safety Policy Editor

## Overview

Safety Policy Editor is a comprehensive GUI application designed for uploading, viewing, and editing safety policy files for various vehicle types and zones. The application supports data from Excel files, enabling users to edit and save the data into a SQLite database. This tool is particularly useful for managing safety policies in environments where different zones and vehicle types require distinct safety measures.

## Features

- **load Files:** Load policy data from Excel files for different zones and sub-zones..
- **View and Edit Data:** Display the uploaded data in a table format for easy viewing and editing.
- **Add and Delete Rows:** Easily add new rows or delete existing rows in the data table.
- **Save to SQLite:** Save the updated data to a SQLite database with a single click.
- **Support for Multiple Zones and Vehicle Types:** Select different zones and vehicle types to work with specific datasets.


## Installation

### Prerequisites

- Python 3.x
- pip (Python package installer)

### Steps
## Install the required dependencies:
     pip install -r requirements.txt

## Running the Application

    Start the application:
       python3 lidar_visualizer.py

## Home Page:

The home page allows you to upload an Excel file and select vehicle types and zones.

## Upload an Excel File:

Click on the upload button to select and upload an Excel file containing safety policy data.
The file name should include the zone information (e.g., "Fork Side.xlsx").

## View and Edit Data:

The uploaded data is displayed in a table format.
Edit data directly within the table.

## Add or Delete Rows:

Use the "Add Row" button to add new rows of data.
Use the "Delete Row" button to remove selected rows of data.

## Save Data to SQLite:

Click the "Save to SQLite" button to save the updated data to a SQLite database.
The data will be saved in the appropriate database file based on the selected zone.
    
    
## Dependencies

    Python 3.x
    pandas
    sqlite3
    PySide2

Install these dependencies using the following command:
       pip install -r requirements.txt

## Detailed Functionality
Uploading Files

The application allows you to upload Excel files. The file name should contain the zone information to automatically detect the zone.
Once a file is uploaded, the data is read using pandas and displayed in a table format.

## Viewing and Editing Data

The data is displayed in a QTableWidget, where each row corresponds to a safety policy entry.
Users can edit the data directly within the table. The Used_For and Speed columns are represented as QComboBox widgets for easy selection of predefined values.

## Adding and Deleting Rows

New rows can be added by clicking the "Add Row" button. Each new row includes default values and dropdowns for Used_For and Speed.
Rows can be deleted by selecting a row and clicking the "Delete Row" button.

## Saving Data to SQLite

The updated data can be saved to a SQLite database. The database file is selected based on the current zone.
Before saving, the application checks if the Speed column exists and adds it if necessary.
The data is then inserted into the appropriate database table.
