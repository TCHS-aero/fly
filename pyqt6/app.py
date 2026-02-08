from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton, QWidget, QCheckBox, QComboBox, QVBoxLayout, QLineEdit, QFileDialog
from PyQt6.QtGui import QIcon, QFont, QPixmap
from PyQt6.QtCore import Qt, QUrl, QSize, QTimer
from PyQt6.QtMultimedia import QAudioOutput, QMediaPlayer
import sys
import os
from fly.core.mission import Mission
from random import randint
import logging


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("draft 1 of drone ui")
        self.setWindowIcon(QIcon("pyqt6/half-baby-half-kid-meme-newgen"))
        self.setGeometry(500,300,1250,750)
        self.setStyleSheet("background-color:white")

        self.waypoint_counter = 0
        self.next_id = 0 
        self.waypoint_x_ID = []

        self.waypoint = QLineEdit(self)
        '''self.waypoint_labels.append(self.waypoint)'''

        self.waypoint.setGeometry(200, 100, 350,50)
        self.waypoint.setPlaceholderText("enter your coordinates! Format: Lat, Lon, Alt")
        self.waypoint.setStyleSheet("color: black")
        self.waypoint.returnPressed.connect(self.parse_text)

        self.error = QLabel("oi man you left an empty parameter bro! fill that in cro!",self)
        self.error.setStyleSheet("color: black")
        self.error.setGeometry(200,50,600,50)
        self.error.hide()

        self.upload_mission = QPushButton("upload preset mission\n (MUST BE CSV FILE)",self)
        self.upload_mission.setGeometry(900,400,200,200)
        self.upload_mission.setStyleSheet("background-color: grey")
        self.upload_mission.clicked.connect(self.file_upload)
        
        self.connect_to_drone = QPushButton('connect to drone', self)
        self.connect_to_drone.setStyleSheet("background-color: grey")
        self.connect_to_drone.setGeometry(900,100,200,50)

        self.takeoff = QPushButton("Takeoff", self)
        self.takeoff.setStyleSheet("background-color: grey")
        self.takeoff.setGeometry(900,160,200,50)

        self.return_to_launch = QPushButton("Return to lauch position", self)
        self.return_to_launch.setStyleSheet("background-color: grey")
        self.return_to_launch.setGeometry(900,220,200,50)

        self.land_drone = QPushButton("Land drone", self)
        self.land_drone.setStyleSheet("background-color: grey")
        self.land_drone.setGeometry(900,280,200, 50)

        '''image = QLabel(self)
        pixmap = QPixmap("pyqt6/teto.jpg")
        image.setGeometry(0,0,500,500)
        image.setPixmap(pixmap)
        image.resize(500,500)'''


        self.biang = QLabel("𰻞", self)
        self.biang.resize(100,100)



    def file_upload(self):
        dialog = QFileDialog()
        dialog.setNameFilter("*.csv")
        dialog.setFileMode(QFileDialog.FileMode.ExistingFile)
        dialogSuccess = dialog.exec()

        if dialogSuccess == 1:
            selectedFiles = dialog.selectedFiles()
            file = selectedFiles[0]
            print(file)
            self.parse_file_upload(file)
            
        else:
            print('user cancelled file selection')


    def parse_file_upload(self, file):
        self.mission = Mission(file) 
        #for waypoint in self.mission.waypoints:
            #lat, lon, alt = self.mission.get_waypoint(waypoint['latitude']), self.mission.get_waypoint(waypoint['longitude']), self.mission.get_waypoint(waypoint['altitude'])
        for i, waypoint_row in enumerate(self.mission.waypoints, 1):  # i=1 to 5
        
            coords = f"{waypoint_row['latitude']},{waypoint_row['longitude']},{waypoint_row['altitude']}"
        
            self.waypoint_counter+=1
            self.waypoint_coord = QLabel(self)
            self.x_label = QPushButton(icon = QIcon("pyqt6/x_mark.jpg"),text="" ,parent=self)
            self.x_label.setCheckable(True)
           
            
            self.x_label.setGeometry(550, (self.waypoint_counter+1)*90, 50,50 )
            self.x_label.setStyleSheet("background-color: red")
            self.x_label.resize(50,50)
            self.x_label.show()
            
            self.waypoint_coord.setGeometry(200, ((self.waypoint_counter+1)*90), 350, 50)
            self.waypoint_coord.setStyleSheet("color: black; background-color: grey")
            self.waypoint_coord.setText(f"Waypoint {self.waypoint_counter}: {coords}")
            self.waypoint_coord.show()

            row_id = self.next_id
            self.x_label.row_id, self.waypoint_coord.row_id = row_id, row_id

            self.x_label.clicked.connect(self.x_clicked)

            self.waypoint_x_ID.append({"id": row_id,"label": self.waypoint_coord,"button": self.x_label})

            self.next_id +=1 





    def parse_text(self):
        text = self.waypoint.text()
        list1= [c.strip() for c in text.split(",")]
        
        if (len(list1) != 3) or any(c=="" for c in list1):
            self.error.setText("oi man you didn't fill in every parameter!/use commas to seperate them!")
            self.error.show()
            QTimer.singleShot(2000, self.error.clear) 
            return
           
        try:
            lat = float(list1[0])
            lon = float(list1[1])
            alt = float(list1[2])
            print(lat, lon, alt)
            coords = f"{lat},{lon},{alt}"
            print(coords)
            self.waypoint_counter +=1
            self.waypoint_coord = QLabel(self)
            self.x_label = QPushButton(icon = QIcon("pyqt6/x_mark.jpg"),text="" ,parent=self)
            self.x_label.setCheckable(True)
            self.x_label.hide()
            
            self.x_label.setGeometry(550, (self.waypoint_counter+1)*90, 50,50 )
            self.x_label.setStyleSheet("background-color: red")
            self.x_label.resize(50,50)
            self.x_label.show()
            
            '''self.waypoint_labels.append(self.waypoint_coord)'''
            self.waypoint_coord.setGeometry(200, ((self.waypoint_counter+1)*90), 350, 50)
            self.waypoint_coord.setStyleSheet("color: black; background-color: grey")
            self.waypoint_coord.setText(f"Waypoint {self.waypoint_counter}: {coords}")
            self.waypoint_coord.show()

            row_id = self.next_id
            self.x_label.row_id, self.waypoint_coord.row_id = row_id, row_id

            self.x_label.clicked.connect(self.x_clicked)

            self.waypoint_x_ID.append({"id": row_id,"label": self.waypoint_coord,"button": self.x_label})

            self.next_id +=1 

        except:
            self.error.setText("oi man you can only have numbers! no letters!")
            self.error.show()
            QTimer.singleShot(2000, self.error.clear) 
            return


    def x_clicked(self):
        btn = self.sender()         
        row_id = btn.row_id   
        for row in self.waypoint_x_ID:
            if row["id"] == row_id:
                row["label"].hide()
                row["label"].setParent(None)

                row["button"].hide()
                row["button"].setParent(None)

                self.waypoint_x_ID.remove(row)
                break

        self.waypoint_counter -=1
def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.statusBar().showMessage("no diabeto roll back to kitchen")
    
    window.show()
    sys.exit(app.exec()) #without this the window default closes. with this the window waits until closure to...close...

if __name__ == "__main__":
    main()

'''
zongle notes:  setEnabled() makes the button clickable, setCheckable allows it to have T/F states, and setChecked finds the current value it's on (return T or F)
button is the interactive object, clicked is when it gets...clicked... and connect links the call to the response 
(in line 28, calling button by clicking on it gets a response from the_button_was_clicked function)
The reason why checked? alts between true and false because whenever we click a button, 
Qt automatically flips the switch to "True". If we do it again, we flip the switch back to "False". 
And also when the button is clicked, Qt passes a check in the_button_was_toggled. 
If there was no parameter, it would simply not run and not return an error. If there was, we'd use it and print it out in the terminal.'''