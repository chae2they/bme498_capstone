import sys
import time
from PyQt5.QtGui import QImage, QPixmap, QMovie, QIcon
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QSizePolicy, QComboBox,  QCheckBox, QScrollArea
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QSize
import cv2
import mediapipe as mp
import numpy as np
import math

# Custom Python Libraries
import serializer

#pose marker constants (technically are opposite to what mediapipe says)
RIGHT_SHOULDER = 11 
LEFT_SHOULDER = 12
RIGHT_ELBOW = 13
LEFT_ELBOW = 14 
RIGHT_WRIST = 15 
LEFT_WRIST = 16 
RIGHT_PINKY = 17 
LEFT_PINKY = 18 
RIGHT_INDEX = 19
LEFT_INDEX = 20 
RIGHT_THUMB = 21
LEFT_THUMB = 22 
RIGHT_HIP = 23
LEFT_HIP = 24

#pose markers for the hands 
HAND_WRIST = 0 
THUMB_CMC = 1 
THUMB_MCP = 2 
THUMB_IP = 3 
THUMB_TIP = 4
INDEX_FINGER_MCP = 5 
INDEX_FINGER_PIP = 6 
INDEX_FINER_DIP = 7 
INDEX_FINGER_TIP = 8 
MIDDLE_FINGER_MCP = 9 
MIDDLE_FINGER_PIP = 10 
MIDDLE_FINGER_DIP = 11
MIDDLE_FINGER_TIP = 12
RING_FINGER_MCP = 13
RING_FINGER_PIP = 14
RING_FINGER_DIP = 15
RING_FINGER_TIP = 16
PINKY_FINGER_MCP = 17
PINKY_FINGER_PIP = 18
PINKY_FINGER_DIP = 19
PINKY_FINGER_TIP = 20

class StartupWindow(QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.setWindowTitle("FESync")
        self.setFixedSize(1500, 1550)  # Static window size
        self.setWindowIcon(QIcon("patient.png"))
        self.setStyleSheet("background-color: #FFFFFF;")

        # Main Layout
        main_layout = QVBoxLayout()
        main_layout.setAlignment(Qt.AlignCenter)

        # Title (Above Images)
        title_label = QLabel("Welcome to FESync")
        title_label.setStyleSheet("font-size: 42px; font-weight: bold; color: #000000; font-family: Arial;")
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)

        # Images Layout
        images_layout = QHBoxLayout()
        images_layout.setAlignment(Qt.AlignCenter)

        # Image 1
        self.image1_label = QLabel()
        self.image1_label.setScaledContents(True)  # Ensure image scales
        pixmap1 = QPixmap("image1.png").scaled(900, 900, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.image1_label.setPixmap(pixmap1)

        # Image 2
        self.image2_label = QLabel()
        self.image2_label.setScaledContents(True)
        pixmap2 = QPixmap("image2.png").scaled(900, 900, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.image2_label.setPixmap(pixmap2)

        images_layout.addWidget(self.image1_label)
        images_layout.addWidget(self.image2_label)
        main_layout.addLayout(images_layout)

        # Instruction Text (Below Images)
        instruction_label = QLabel("""
            <h2 style="font-size: 30px;color: #000000; text-align: center; border-bottom: 2px solid #CCCCCC; padding-bottom: 16px;font-family: Arial;">
                Best Practices
            </h2>

            <ul style="font-size: 24px; line-height: 2; color: #222222; padding-left: 18px;font-family: Arial;">
                <li><b style="color: #000000;">Electrode Placement:</b> Ensure electrodes are applied as shown in the diagram above for the correct muscle activation.</li>
                
                <li><b style="color: #000000;">Camera Positioning:</b> Place the camera directly in front of the therapist. Ensure the hip, shoulder, arm, and hands are clearly visible at all times.</li>
                
                <li><b style="color: #000000;">Lighting Conditions:</b> Use good lighting to enhance tracking accuracy. Avoid dim environments or strong backlighting, which may interfere with movement detection.</li>

                <li><b style="color: #000000;">Clothing:</b> The therapist should wear form-fitting clothing to ensure tracking accuracy. Loose or baggy clothing may obscure key landmarks and reduce accuracy.</li>
            </ul>
        """)
        instruction_label.setWordWrap(True)
        instruction_label.setAlignment(Qt.AlignLeft)
        instruction_label.setStyleSheet("""
            font-size: 18px;
            padding: 10px;
            color: #333333;
            background: white;
            border-radius: 10px;
        """)

        # Fixed-size container to keep the same height as before (without scrollbar)
        instruction_container = QWidget()
        instruction_layout = QVBoxLayout()
        instruction_layout.addWidget(instruction_label)
        instruction_container.setLayout(instruction_layout)
        instruction_container.setFixedHeight(325)  # Same height as before

        main_layout.addWidget(instruction_container)


        # OK Button (Centered Below Text)
        self.ok_button = QPushButton("OK")
        self.ok_button.setFixedSize(300, 75)
        self.ok_button.clicked.connect(self.proceed_to_main)
        self.ok_button.setStyleSheet(
            """
            QPushButton {
                background-color: #3399FF;  /* Green */
                color: white;               /* White text */
                font-size: 18px;
                font-weight: bold;
                padding: 10px;
                border-radius: 10px;  /* Rounded corners */
                border: 2px solid #3399FF;
            }
            
            QPushButton:hover {
                background-color: #0080FF; /* Slightly darker green when hovered */
            }

            QPushButton:pressed {
                background-color: #0066CC; /* Darker green when pressed */
            }
            """
        )

        main_layout.addWidget(self.ok_button, alignment=Qt.AlignCenter)

        self.setLayout(main_layout)
    
    def proceed_to_main(self):
        self.main_window.show()
        self.close()

class MainWindow(QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        
        self.setWindowIcon(QIcon("patient.png"))
        self.setWindowTitle("FESync")
        self.setStyleSheet("background-color: #FFFFFF;")
        self.resize(1500, 1000)

        # Main Horizontal Layout: (Left: Video, Right: Controls)
        self.MainLayout = QHBoxLayout(self)

        # Left side (video feed)
        self.FeedLabel = QLabel()
        self.FeedLabel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.FeedLabel.setScaledContents(True)
        self.FeedLabel.setMinimumSize(800, 600)
        self.FeedLabel.setStyleSheet("border: 2px solid #CCCCCC; border-radius: 10px; background-color: #F8F8F8;")
        self.MainLayout.addWidget(self.FeedLabel)

        # Right side (controls)
        self.ControlLayout = QVBoxLayout()

        # Checkboxes (at the top)
        self.StatusLayout = QVBoxLayout()
        self.StatusLayout.setAlignment(Qt.AlignTop)

        title_label = QLabel("Stimulation Channels")
        title_label.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: #000000;
            font-family: Arial;
            text-align: center;
        """)
        title_label.setAlignment(Qt.AlignLeft)
        self.StatusLayout.addWidget(title_label)
        
        self.checkbox_style = """
        QCheckBox {
            font-size: 18px;
            font-weight: bold;
            color: #444444;
            padding: 10px;
            font-family: Arial;
            border-radius: 8px;
            background: #FFFFFF;
            border: 2px solid #CCCCCC;
        }

        QCheckBox:hover {
            color: #0080FF;
            border: 2px solid #0080FF;
        }

        QCheckBox::indicator {
            width: 24px;
            height: 24px;
        }

        QCheckBox::indicator:checked {
            background-color: #3399FF;
            border: 2px solid #0080FF;
        }

        QCheckBox::indicator:unchecked {
            background-color: #FFFFFF;
            border: 2px solid #CCCCCC;
        }
    """

        self.checkbox_style_active = """
        QCheckBox {
            font-size: 18px;
            font-weight: bold;
            color: #FFFFFF;
            padding: 10px;
            font-family: Arial;
            border-radius: 8px;
            background: #7DCEA0;  /* Green */
            border: 2px solid #1E8449;  /* Darker green */
        }

        QCheckBox:hover {
            background: #2229954;
            border: 2px solid #196f3D;
        }

        QCheckBox::indicator {
            width: 24px;
            height: 24px;
        }

        QCheckBox::indicator:checked {
            background-color: #229954;
            border: 2px solid #1E8449;
        }

        QCheckBox::indicator:checked:hover {
            background-color: #1E8449;
            border: 2px solid #196F3D;
        }
    """
        self.LateralDeltoidCheckBox = QCheckBox("Channel One (Shoulder Abduction)")
        self.LateralDeltoidCheckBox.setChecked(False)
        self.LateralDeltoidCheckBox.setStyleSheet(self.checkbox_style)
        self.StatusLayout.addWidget(self.LateralDeltoidCheckBox)

        self.BicepCheckBox = QCheckBox("Channel Two (Elbow Extension)")
        self.BicepCheckBox.setChecked(False)
        self.BicepCheckBox.stateChanged.connect(self.ToggleBicepStatus)
        self.BicepCheckBox.setStyleSheet(self.checkbox_style)
        self.StatusLayout.addWidget(self.BicepCheckBox)

        self.WristExtensionCheckBox = QCheckBox("Channel Three (Wrist Extension)")
        self.WristExtensionCheckBox.setChecked(False)
        self.WristExtensionCheckBox.setStyleSheet(self.checkbox_style)
        self.StatusLayout.addWidget(self.WristExtensionCheckBox)
        
        self.FingerFlexCheckBox = QCheckBox("Channel Four (Finger Extension)")
        self.FingerFlexCheckBox.setChecked(False)
        self.FingerFlexCheckBox.setStyleSheet(self.checkbox_style)
        self.StatusLayout.addWidget(self.FingerFlexCheckBox)
    
        self.ThumbAbductionCheckBox = QCheckBox("Channel Five (Thumb Flexion & Abduction)")
        self.ThumbAbductionCheckBox.setChecked(False)
        self.ThumbAbductionCheckBox.setStyleSheet(self.checkbox_style)
        self.StatusLayout.addWidget(self.ThumbAbductionCheckBox)


        self.ControlLayout.addLayout(self.StatusLayout)

        # Buttons (below checkboxes)
        button_style = """
            QPushButton {
                background-color: #3399FF;
                color: white;
                font-size: 18px;
                font-weight: bold;
                padding: 12px;
                border-radius: 10px;
                border: 2px solid #3399FF;
                font-family: Arial;
            }
            QPushButton:hover {
                background-color: #0080FF;
            }
            QPushButton:pressed {
                background-color: #0066CC;
            }
        """

        self.ButtonLayout = QVBoxLayout()
        self.TrackingButton = QPushButton("Start Tracking")
        self.TrackingButton.setStyleSheet(button_style)
        self.TrackingButton.clicked.connect(self.ToggleTracking)
        self.ButtonLayout.addWidget(self.TrackingButton)

        # Dropdown for tracking mode
        self.TrackingModeDropdown = QComboBox()
        self.TrackingModeDropdown.addItems(["Right Arm & Hand", "Left Arm & Hand"])
        self.TrackingModeDropdown.setStyleSheet("""
            QComboBox {
                font-size: 18px;
                padding: 8px;
                border: 2px solid #888888;
                border-radius: 10px;
                background-color: #f9f9f9;
                font-family: Arial;
                selection-background-color: #4CAF50; /* Green highlight */
            }
            
            QComboBox:hover {
                border: 2px solid #4CAF50; /* Green border on hover */
            }

            QComboBox:focus {
                border: 2px solid #0078D7; /* Blue border when focused */
                background-color: #FFFFFF;
            }

            QComboBox QAbstractItemView {
                background-color: white;
                border-radius: 8px;
                border: 1px solid #CCCCCC;
                selection-background-color: #4CAF50; /* Green highlight */
                selection-color: white;
                font-size: 16px;
            }

            QComboBox::drop-down {
                border: none;
                width: 40px;
            }

            QComboBox::down-arrow {
                image: url(down_arrow.png); /* Custom dropdown arrow */
                width: 16px;
                height: 16px;
            }
        """)
        self.TrackingModeDropdown.currentIndexChanged.connect(self.UpdateTrackingMode)
        self.ButtonLayout.addWidget(self.TrackingModeDropdown)
        self.ControlLayout.addLayout(self.ButtonLayout)

        # FPS Display (at the bottom)
        self.FPSLayout = QVBoxLayout()
        self.FPSLabel = QLabel("FPS: Calculating...")
        self.FPSLabel.setAlignment(Qt.AlignCenter)
        self.FPSLabel.setStyleSheet("""
        QLabel {
            font-size: 20px;
            color: #000000;  /* Bright blue text */
            font-family: Arial, sans-serif;
            padding: 5px;
        }
    """)
        self.FPSLayout.addWidget(self.FPSLabel, alignment=Qt.AlignTop)
        self.ControlLayout.addLayout(self.FPSLayout)

        # Add ControlLayout (Right Side) to Main Layout
        self.MainLayout.addLayout(self.ControlLayout)
        self.setLayout(self.MainLayout)

        self.Worker1 = Worker1(self.LateralDeltoidCheckBox, self.BicepCheckBox, self.WristExtensionCheckBox, 
                               self.FingerFlexCheckBox, self.ThumbAbductionCheckBox)
        self.Worker1.ImageUpdate.connect(self.ImageUpdateSlot)
        self.Worker1.FPSUpdate.connect(self.UpdateFPS)
        self.Worker1.BicepStatusUpdate.connect(self.UpdateBicepStatus)
        self.Worker1.FingerFlexStatusUpdate.connect(self.UpdateFingerFlexStatus)
        self.Worker1.ThumbAbductionStatusUpdate.connect(self.UpdateThumbAbductionStatus)
        self.Worker1.WristExtensionStatusUpdate.connect(self.UpdateWristExtensionStatus)
        self.Worker1.LateralDeltoidStatusUpdate.connect(self.UpdateLateralDeltoidStatus)
        self.Worker1.start()

        # Handle window close event
        self.closeEvent = self.handle_close_event
    
    def ToggleTracking(self):
        """ Toggle tracking on/off and update button text """
        if self.Worker1.TrackingActive:
            self.Worker1.stop_tracking()
            self.TrackingButton.setText("Start Tracking")
            self.UpdateBicepStatus("down")
        else:
            self.Worker1.start_tracking()
            self.TrackingButton.setText("Stop Tracking")

    def resizeEvent(self, event):
        self.update_feed_size()
    
    def update_feed_size(self):
        if hasattr(self, 'FeedLabel') and self.FeedLabel.pixmap():
            self.FeedLabel.setPixmap(self.FeedLabel.pixmap().scaled(self.FeedLabel.width(), self.FeedLabel.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
    
    def ImageUpdateSlot(self, Image):
        pixmap = QPixmap.fromImage(Image)
        self.FeedLabel.setPixmap(pixmap.scaled(self.FeedLabel.width(), self.FeedLabel.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
    
    def UpdateFPS(self, fps):
        self.FPSLabel.setText(f"FPS: {fps:.2f}")

    def ToggleBicepStatus(self):
        """ If the checkbox is unchecked, force status to 'down' """
        if not self.BicepCheckBox.isChecked():
            self.UpdateBicepStatus("down")

    def UpdateBicepStatus(self, status):
        """ Update checkbox color based on bicep status, only if it is checked """
        if self.BicepCheckBox.isChecked():
            if status == "up":
                self.BicepCheckBox.setStyleSheet(self.checkbox_style_active)
            else:
                self.BicepCheckBox.setStyleSheet(self.checkbox_style)
        else:
            self.BicepCheckBox.setStyleSheet(self.checkbox_style)

    def UpdateFingerFlexStatus(self, status):
        if self.FingerFlexCheckBox.isChecked():   
            if status == "flexed":
                self.FingerFlexCheckBox.setStyleSheet(self.checkbox_style_active)
            else:
                self.FingerFlexCheckBox.setStyleSheet(self.checkbox_style)
        else: 
                self.FingerFlexCheckBox.setStyleSheet(self.checkbox_style)
            
    def UpdateThumbAbductionStatus(self, status):
        if self.ThumbAbductionCheckBox.isChecked():   
            if status == "flexed":
                self.ThumbAbductionCheckBox.setStyleSheet(self.checkbox_style_active)
            else:
                self.ThumbAbductionCheckBox.setStyleSheet(self.checkbox_style)
        else: 
                self.ThumbAbductionCheckBox.setStyleSheet(self.checkbox_style)

    def UpdateWristExtensionStatus(self, status):
        if self.WristExtensionCheckBox.isChecked():   
            if status == "extended":
                self.WristExtensionCheckBox.setStyleSheet(self.checkbox_style_active)
            else:
                self.WristExtensionCheckBox.setStyleSheet(self.checkbox_style)
        else: 
                self.WristExtensionCheckBox.setStyleSheet(self.checkbox_style)

    def UpdateLateralDeltoidStatus(self, status):
        if self.LateralDeltoidCheckBox.isChecked():   
            if status == "extended":
                self.LateralDeltoidCheckBox.setStyleSheet(self.checkbox_style_active)
            else:
                self.LateralDeltoidCheckBox.setStyleSheet(self.checkbox_style)
        else: 
                self.LateralDeltoidCheckBox.setStyleSheet(self.checkbox_style)
    
    def StartTracking(self):
        self.Worker1.start_tracking()
    
    def StopTracking(self):
        self.Worker1.stop_tracking()
    
    def UpdateTrackingMode(self, index):
        self.Worker1.set_tracking_mode(index == 0)
    
    def handle_close_event(self, event):
        self.Worker1.stop()
        event.accept()

class Worker1(QThread):

    ImageUpdate = pyqtSignal(QImage)
    FPSUpdate = pyqtSignal(float)
    BicepStatusUpdate = pyqtSignal(str)
    FingerFlexStatusUpdate = pyqtSignal(str)
    ThumbAbductionStatusUpdate = pyqtSignal(str)
    WristExtensionStatusUpdate = pyqtSignal(str)
    LateralDeltoidStatusUpdate = pyqtSignal(str)
    
    def __init__(self, lateral_deltoid_checkbox, bicep_checkbox, wrist_extension_checkbox, 
                finger_flex_checkbox, thumb_abduction_checkbox):
        super().__init__()
        self.lateral_deltoid_checkbox = lateral_deltoid_checkbox
        self.bicep_checkbox = bicep_checkbox
        self.wrist_extension_checkbox = wrist_extension_checkbox
        self.finger_flex_checkbox = finger_flex_checkbox
        self.thumb_abduction_checkbox = thumb_abduction_checkbox
        self.Capture = cv2.VideoCapture(0)
        self.ThreadActive = True
        self.TrackingActive = False
        self.TrackRight = True  # Default to tracking right arm and hand
        self.mpPose = mp.solutions.pose.Pose(min_detection_confidence=0.8, min_tracking_confidence=0.8)
        self.mpHands = mp.solutions.hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.8)
        self.mpDraw = mp.solutions.drawing_utils
        self.prev_time = time.time()
        self.armResults = None
        self.handResults = None
        self.bicep_stage = "down"
        self.thumb_stage = "extended"
        self.fingers_flex = "extended"
        self.wrist_stage = "relaxed"
        self.lateral_deltoid_stage = "relaxed"
        #intialize stm
        #self.uart_serializer = serializer.STM32_Serializer(start_sig={1:0, 2:0, 3:0, 4:0, 5:0, 6:0, 7:0, 8:0}, send_freq=30)
        #self.uart_serializer.start()

    def run(self):
        while not self.Capture.isOpened():
            pass
        while self.ThreadActive:
            ret, frame = self.Capture.read()
            if ret:
                Image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                FlippedImage = cv2.flip(Image, 1)
                h, w, _ = FlippedImage.shape
                if self.TrackingActive:
                    self.armResults = self.mpPose.process(FlippedImage)
                    if self.armResults.pose_landmarks:
                
                        angle = self.detect_bicep_curl()
                        self.detect_deltoid()
                        if self.TrackRight:
                            elbow = self.armResults.pose_landmarks.landmark[RIGHT_ELBOW] 
                        else:
                            elbow = self.armResults.pose_landmarks.landmark[LEFT_ELBOW]

                        elbow_px = (int(elbow.x * w), int(elbow.y * h))
                        cv2.putText(FlippedImage, f"{angle:.2f}", (elbow_px[0] - 20, elbow_px[1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

                        arm_points = [11, 13, 15] if self.TrackRight else [12, 14, 16]  # Right or Left arm tracking
                        for idx in arm_points:
                            landmark = self.armResults.pose_landmarks.landmark[idx]
                            h, w, _ = FlippedImage.shape
                            cx, cy = int(landmark.x * w), int(landmark.y * h)
                            cv2.circle(FlippedImage, (cx, cy), 5, (0, 0, 255), -1)  # Blue Points 
                            cv2.putText(FlippedImage, f"({cx},{cy})", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)  # White text
                        for i in range(len(arm_points)-1):
                            p1 = arm_points[i]
                            p2 = arm_points[i+1]
                            cx1, cy1 = int(self.armResults.pose_landmarks.landmark[p1].x * w), int(self.armResults.pose_landmarks.landmark[p1].y * h)
                            cx2, cy2 = int(self.armResults.pose_landmarks.landmark[p2].x * w), int(self.armResults.pose_landmarks.landmark[p2].y * h)
                            cv2.line(FlippedImage, (cx1, cy1), (cx2, cy2), (255, 255, 255), 2)  # White lines for arms
                    
                    self.handResults = self.mpHands.process(FlippedImage)
                    self.detect_finger_flexion()
                    self.detect_thumb_abduction()
                    self.detect_wrist_extension()
                    if self.handResults.multi_hand_landmarks and self.handResults.multi_handedness:
                        for i, handLms in enumerate(self.handResults.multi_hand_landmarks):
                            handedness = self.handResults.multi_handedness[i].classification[0].label
                            if (self.TrackRight and handedness == "Right") or (not self.TrackRight and handedness == "Left"):
                                self.mpDraw.draw_landmarks(FlippedImage, handLms, mp.solutions.hands.HAND_CONNECTIONS)
                
                curr_time = time.time()
                fps = 1 / (curr_time - self.prev_time)
                self.prev_time = curr_time
                self.FPSUpdate.emit(fps)
                ConvertToQtFormat = QImage(FlippedImage.data, FlippedImage.shape[1], FlippedImage.shape[0], FlippedImage.strides[0], QImage.Format_RGB888)
                self.ImageUpdate.emit(ConvertToQtFormat)

                # send to the board
                #note to self update to make it so that it only updates when checked
                buffer_to_stm = {}
                if self.TrackingActive: 
                    if self.lateral_deltoid_checkbox.isChecked():
                        if self.lateral_deltoid_stage == "relaxed": 
                            buffer_to_stm[1] = 0 
                        else:
                            buffer_to_stm[1] = 1
                    else:
                        buffer_to_stm[1] = 0 
                    if self.bicep_checkbox.isChecked():
                        if self.bicep_stage == "down": 
                            buffer_to_stm[2] = 0 
                        else:
                            buffer_to_stm[2]= 1
                    else: 
                        buffer_to_stm[2] = 0
                    if self.wrist_extension_checkbox.isChecked():
                        if self.wrist_stage == "relaxed":
                            buffer_to_stm[3] = 0 
                        else:
                            buffer_to_stm[3] = 1 
                    else: 
                        buffer_to_stm[3] = 0
                    if self.finger_flex_checkbox.isChecked():
                        if self.fingers_flex == "extended":
                            buffer_to_stm[4] = 0
                        else: 
                            buffer_to_stm[4] = 1
                    else:
                        buffer_to_stm[4] = 0
                    if self.thumb_abduction_checkbox.isChecked():
                        if self.thumb_stage == "extended":
                            buffer_to_stm[5] = 0
                        else: 
                            buffer_to_stm[5] = 1
                    else:
                        buffer_to_stm[5] = 0
                    buffer_to_stm[6] = 0
                    buffer_to_stm[7] = 0
                    buffer_to_stm[8] = 0 
                else: 
                    buffer_to_stm[1] = 0 
                    buffer_to_stm[2] = 0
                    buffer_to_stm[3] = 0
                    buffer_to_stm[4] = 0
                    buffer_to_stm[5] = 0
                    buffer_to_stm[6] = 0
                    buffer_to_stm[7] = 0
                    buffer_to_stm[8] = 0
                print(buffer_to_stm)  
                #assuming this takes care of timing 
                #self.uart_serializer.update_signal(buffer_to_stm)           
        self.Capture.release()
    
    def start_tracking(self):
        self.TrackingActive = True
    
    def stop_tracking(self):
        self.TrackingActive = False
    
    def set_tracking_mode(self, track_right):
        self.TrackRight = track_right
    
    def stop(self):
        # uart seralizer end
        # self.uart_serializer.end()
        self.ThreadActive = False
        self.quit()
        self.wait()

    def detect_bicep_curl(self): 
        if self.armResults.pose_landmarks is None:
            return 0

        if self.TrackRight:
            shoulder = [self.armResults.pose_landmarks.landmark[RIGHT_SHOULDER].x, self.armResults.pose_landmarks.landmark[RIGHT_SHOULDER].y]
            elbow = [self.armResults.pose_landmarks.landmark[RIGHT_ELBOW].x, self.armResults.pose_landmarks.landmark[RIGHT_ELBOW].y]
            wrist = [self.armResults.pose_landmarks.landmark[RIGHT_WRIST].x, self.armResults.pose_landmarks.landmark[RIGHT_WRIST].y]
        else:
            shoulder = [self.armResults.pose_landmarks.landmark[LEFT_SHOULDER].x, self.armResults.pose_landmarks.landmark[LEFT_SHOULDER].y]
            elbow = [self.armResults.pose_landmarks.landmark[LEFT_ELBOW].x, self.armResults.pose_landmarks.landmark[LEFT_ELBOW].y]
            wrist = [self.armResults.pose_landmarks.landmark[LEFT_WRIST].x, self.armResults.pose_landmarks.landmark[LEFT_WRIST].y]

        angle = calculate_angle(shoulder, elbow, wrist)
        if angle < 45 and self.bicep_stage == "down":
            self.bicep_stage = "up"
        elif angle >= 45:
            self.bicep_stage = "down"
        self.BicepStatusUpdate.emit(self.bicep_stage)
        return angle

    def detect_finger_flexion(self): 
        
        if not self.TrackingActive:
            self.fingers_flex = "extended"
            self.FingerFlexStatusUpdate.emit(self.fingers_flex)  # Reset checkbox
            return
        
        if self.handResults is None or self.handResults.multi_hand_landmarks is None: 
            self.fingers_flex = "extended"
            self.FingerFlexStatusUpdate.emit(self.fingers_flex)
            return 

        if self.handResults is None or self.handResults.multi_hand_landmarks is None:
            self.fingers_flex = "extended"
            self.FingerFlexStatusUpdate.emit(self.fingers_flex)  # Reset checkbox
            return
            
        flexion_status = {
        "index": "extended", "middle": "extended", "ring": "extended", "pinky": "extended"  
        }

        FLEXION_THRESHOLD = 90  

        for i, handLms in enumerate(self.handResults.multi_hand_landmarks):
            handedness = self.handResults.multi_handedness[i].classification[0].label

            # Ensure we only process the correct hand
            if (self.TrackRight and handedness != "Right") or (not self.TrackRight and handedness != "Left"):
                continue  # Skip if it's the wrong hand

            fingers = {
                "index": [handLms.landmark[INDEX_FINGER_MCP], handLms.landmark[INDEX_FINGER_PIP], handLms.landmark[INDEX_FINGER_TIP]],
                "middle": [handLms.landmark[MIDDLE_FINGER_MCP], handLms.landmark[MIDDLE_FINGER_PIP], handLms.landmark[MIDDLE_FINGER_TIP]],
                "ring": [handLms.landmark[RING_FINGER_MCP], handLms.landmark[RING_FINGER_PIP], handLms.landmark[RING_FINGER_TIP]],
                "pinky": [handLms.landmark[PINKY_FINGER_MCP], handLms.landmark[PINKY_FINGER_PIP], handLms.landmark[PINKY_FINGER_TIP]]
            }

            for finger, points in fingers.items():
                angle = calculate_angle(*[[p.x, p.y] for p in points])
                if angle < FLEXION_THRESHOLD:
                    flexion_status[finger] = "flexed"

        if all(status == "flexed" for status in flexion_status.values()):
            self.fingers_flex = "flexed"
            self.FingerFlexStatusUpdate.emit(self.fingers_flex)
        else:
            self.fingers_flex = "extended"
            self.FingerFlexStatusUpdate.emit(self.fingers_flex)
        return

    def detect_thumb_abduction(self): 
        if not self.TrackingActive:
            self.thumb_stage = "extended"
            self.ThumbAbductionStatusUpdate.emit(self.thumb_stage)  # Reset checkbox
            return
        
        if self.handResults is None or self.handResults.multi_hand_landmarks is None:
            self.thumb_stage = "extended" 
            self.ThumbAbductionStatusUpdate.emit(self.thumb_stage)
            return 

        for i, handLms in enumerate(self.handResults.multi_hand_landmarks):
            handedness = self.handResults.multi_handedness[i].classification[0].label

            # Ensure we only process the correct hand
            if (self.TrackRight and handedness != "Right") or (not self.TrackRight and handedness != "Left"):
                continue  # Skip this hand if it's the wrong side

            # Get thumb joint positions
            thumb_mcp = [handLms.landmark[THUMB_MCP].x, handLms.landmark[THUMB_MCP].y]
            thumb_ip = [handLms.landmark[THUMB_IP].x, handLms.landmark[THUMB_IP].y]
            thumb_tip = [handLms.landmark[THUMB_TIP].x, handLms.landmark[THUMB_TIP].y]
            hand_wrist = [handLms.landmark[THUMB_CMC].x, handLms.landmark[THUMB_CMC].y]
            

            # Compute angle using the corrected order
            angle = calculate_angle(thumb_mcp, thumb_ip, thumb_tip)

            angle2 = calculate_angle(hand_wrist, thumb_mcp, thumb_tip)

            # Threshold-based classification
            if angle <= 155 :
                self.thumb_stage = "flexed"
            elif angle > 155 and self.thumb_stage == "flexed":
                self.thumb_stage = "extended"
            self.ThumbAbductionStatusUpdate.emit(self.thumb_stage)
        return  

    def detect_wrist_extension(self): 
        if not self.TrackingActive:
            self.wrist_stage = "relaxed"
            self.WristExtensionStatusUpdate.emit(self.wrist_stage)
            return

        if self.armResults.pose_landmarks is None or self.handResults is None or self.handResults.multi_hand_landmarks is None:
            self.wrist_stage = "relaxed"
            self.WristExtensionStatusUpdate.emit(self.wrist_stage)
            return

        # Get wrist and elbow position from pose tracking
        if self.TrackRight:
            wrist = np.array([
                self.armResults.pose_landmarks.landmark[RIGHT_WRIST].x,
                self.armResults.pose_landmarks.landmark[RIGHT_WRIST].y,
                self.armResults.pose_landmarks.landmark[RIGHT_WRIST].z
            ])
        else:
            wrist = np.array([
                self.armResults.pose_landmarks.landmark[LEFT_WRIST].x,
                self.armResults.pose_landmarks.landmark[LEFT_WRIST].y,
                self.armResults.pose_landmarks.landmark[LEFT_WRIST].z
            ])

        # Find the knuckle (index finger MCP) joint from the hand landmarks
        for i, handLms in enumerate(self.handResults.multi_hand_landmarks):
            handedness = self.handResults.multi_handedness[i].classification[0].label
            
            if (self.TrackRight and handedness != "Right") or (not self.TrackRight and handedness != "Left"):
                continue

            knuckle = np.array([
                handLms.landmark[INDEX_FINGER_MCP].x,
                handLms.landmark[INDEX_FINGER_MCP].y,
                handLms.landmark[INDEX_FINGER_MCP].z
            ])
            tip = np.array([
                handLms.landmark[INDEX_FINGER_TIP].x,
                handLms.landmark[INDEX_FINGER_TIP].y,
                handLms.landmark[INDEX_FINGER_TIP].z
            ])


            wrist_to_knuckle_vector = knuckle - wrist
            tip_to_knuckle_vector = tip-knuckle

            cross_product2 = np.cross(tip_to_knuckle_vector[:2], wrist_to_knuckle_vector[:2])
            if not self.TrackRight:
                cross_product2 = -cross_product2  # Flip sign for left hand
            movement2 = "extended" if cross_product2 < 0 else "relaxed"

            angle2 = calculate_angle(tip, knuckle, wrist)


            # Use a lower threshold for wrist extension detection
            EXTENSION_THRESHOLD = 150  
            if movement2 == "extended":
                self.wrist_stage = "extended"
            else:
                self.wrist_stage = "relaxed"
            
            self.WristExtensionStatusUpdate.emit(self.wrist_stage)
        return
    
    
    def detect_deltoid(self): 
        if self.armResults.pose_landmarks is None:
            return  # Ensure pose landmarks exist before processing

        # Get the tracked arm landmarks
        if self.TrackRight:
            shoulder = self.armResults.pose_landmarks.landmark[RIGHT_SHOULDER]
            elbow = self.armResults.pose_landmarks.landmark[RIGHT_ELBOW]
            hip = self.armResults.pose_landmarks.landmark[RIGHT_HIP]
            opposite_shoulder = self.armResults.pose_landmarks.landmark[LEFT_SHOULDER]
        else:
            shoulder = self.armResults.pose_landmarks.landmark[LEFT_SHOULDER]
            elbow = self.armResults.pose_landmarks.landmark[LEFT_ELBOW]
            hip = self.armResults.pose_landmarks.landmark[LEFT_HIP]
            opposite_shoulder = self.armResults.pose_landmarks.landmark[RIGHT_SHOULDER]

        # Convert to numpy arrays (Ignore Z for stability)
        shoulder_point = np.array([shoulder.x, shoulder.y])  
        elbow_point = np.array([elbow.x, elbow.y])  
        hip_point = np.array([hip.x, hip.y])  
        opposite_shoulder_point = np.array([opposite_shoulder.x, opposite_shoulder.y])  

        angle2 = calculate_angle(opposite_shoulder_point, shoulder_point, elbow_point)  # Shoulder-Opposite Shoulder-Elbow

        if angle2 > 135: 
            self.lateral_deltoid_stage = "extended"
            self.LateralDeltoidStatusUpdate.emit(self.lateral_deltoid_stage)

        elif angle2 <= 135: 
            self.lateral_deltoid_stage = "relaxed"
            self.LateralDeltoidStatusUpdate.emit(self.lateral_deltoid_stage)

def calculate_angle(a,b,c):
    a = np.array(a)
    b = np.array(b)
    c = np.array(c) 

    radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
    angle = np.abs(radians*180.0/np.pi)

    if angle>180.0:
        angle = 360-angle
    
    return angle

def calculate_angle_360(a, b, c):
    """
    Calculate the angle between three points in 3D space with better accuracy.
    """
    a = np.array(a)  # Hip or opposite shoulder
    b = np.array(b)  # Shoulder (pivot point)
    c = np.array(c)  # Elbow

    # Vectors
    ba = a - b  # Vector BA
    bc = c - b  # Vector BC

    # Normalize vectors
    ba /= np.linalg.norm(ba)
    bc /= np.linalg.norm(bc)

    # Compute angle using dot product
    dot_product = np.dot(ba, bc)
    angle = np.degrees(np.arccos(np.clip(dot_product, -1.0, 1.0)))  # Clip for numerical stability

    # Use cross product to determine directionality
    cross_product = np.cross(ba, bc)
    if cross_product[2] < 0:  # If cross product Z component is negative, adjust angle
        angle = 360 - angle

    return angle

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    startup_window = StartupWindow(main_window)
    startup_window.show()
    sys.exit(app.exec_())
