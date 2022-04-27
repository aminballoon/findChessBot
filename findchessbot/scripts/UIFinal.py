#!/usr/bin/env python3
from re import X
from PyQt5 import QtCore, QtGui, QtWidgets
import time
import sys, collections
from threading import Thread
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from ai.srv import AIComputeAndPlay
from findchessbot.srv import Capture, Castling, PicknPlace, PromotePawn, McuMode
from findchessbot.msg import JogJoint, JogLinear
FEN='rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1'
class GuiNode(Node):
    def __init__(self):
        super().__init__("simple_gui_node")
        self.PickNPlace = self.create_client(PicknPlace, '/findchessbot/picknplace')
        self.PickNPlaceReq = PicknPlace.Request()
        self.Capture = self.create_client(Capture, '/findchessbot/capture')
        self.CaptureReq = Capture.Request()
        self.Promote = self.create_client(PromotePawn, '/findchessbot/promotepawn')
        self.PromoteReq = PromotePawn.Request()
        self.Castling = self.create_client(Castling, '/findchessbot/castling')
        self.CastlingReq = PicknPlace.Request()
        self.AiComputeAndPlay = self.create_client(AIComputeAndPlay, '/ai/AIComputeAndPlay')
        self.aiComputeAndPlay = AIComputeAndPlay.Request()
        self.mode = self.create_client(McuMode, '/findchessbot/mcu_mode')
        self.modereq = McuMode.Request()
        self.JJ = self.create_publisher(JogJoint, '/findchessbot/JointJog', 10)
        # self.JJreq = JogJoint.Request()
        self.LJ = self.create_publisher(JogLinear, '/findchessbot/LinearJog', 10)
        # self.LJreq = JogLinear.Request()
    def pickplace(self,pick,place,mode):
        if mode == 0:
            self.PickNPlaceReq.chess_square_pick = pick
            self.PickNPlaceReq.chess_square_place = place
            self.future = self.PickNPlace.call_async(self.PickNPlaceReq)
            self.get_logger().info('sent_move_chess_piece_request')
        if mode == 1:
            self.CaptureReq.chess_square_eat = place
            self.CaptureReq.chess_square_pick = pick
            self.CaptureReq.chess_square_place = place
            self.future = self.Capture.call_async(self.CaptureReq)
            self.get_logger().info('capture_request')
        if mode == 2:
            self.PromoteReq.chess_square_pick_pawn = pick
            self.PromoteReq.chess_square_pawn_to_trash = "99"
            self.PromoteReq.chess_square_pick_queen = "00"
            self.PromoteReq.chess_square_place = place
            self.future = self.Promote.call_async(self.PromoteReq)
            self.get_logger().info('Promote_request')
        if mode == 3:############################################
            if pick=="e1" and place=="g1":
                self.CastlingReq.chess_square_pick_king = 'e1'
                self.CastlingReq.chess_square_place_king = 'g1'
                self.CastlingReq.chess_square_pick_rook = 'h1'
                self.CastlingReq.chess_square_place_rook = 'f1'
            if pick=="e1" and place=="c1":
                self.CastlingReq.chess_square_pick_king = 'e1'
                self.CastlingReq.chess_square_place_king = 'c1'
                self.CastlingReq.chess_square_pick_rook = 'a1'
                self.CastlingReq.chess_square_place_rook = 'd1'
            if pick=="e8" and place=="g8":
                self.CastlingReq.chess_square_pick_king = 'e8'
                self.CastlingReq.chess_square_place_king = 'g8'
                self.CastlingReq.chess_square_pick_rook = 'h1'
                self.CastlingReq.chess_square_place_rook = 'f1'
            if pick=="e8" and place=="c8":
                self.CastlingReq.chess_square_pick_king = 'e1'
                self.CastlingReq.chess_square_place_king = 'c1'
                self.CastlingReq.chess_square_pick_rook = 'a1'
                self.CastlingReq.chess_square_place_rook = 'd1'
            self.future = self.Castling.call_async(self.CastlingReq)
            self.get_logger().info('castling_request')
    def sendfen(self,fen):
        self.aiComputeAndPlay.fen_in = FEN
        self.future = self.AiComputeAndPlay.call_async(self.aiComputeAndPlay)
        self.get_logger().info('compute n play')
    def modechange(self,mode):
        self.modereq.update_mode = mode
        self.future = self.mode.call_async(self.modereq)
        self.get_logger().info('mode'+str(mode))
    def sendJJ(self,a,b,c,d):
        self.JJreq.angular_velocity_q1 = a
        self.JJreq.linear_velocity_q2 = b
        self.JJreq.angular_velocity_q3 = c
        self.JJreq.angular_velocity_q4 = d
        self.future = self.JJ.call_async(self.JJreq)
        self.get_logger().info('JJ')
    def sendLJ(self,a,b,c,d):
        self.LJreq.linear_velocity_x = a
        self.LJreq.linear_velocity_y = b
        self.LJreq.linear_velocity_z = c
        self.LJreq.angular_velocity_yaw = d 
        self.future = self.LJ.call_async(self.LJreq)
        self.get_logger().info('LJ')


class Ui_MainWindow(QtWidgets.QMainWindow,object):
    def __init__(self, ros_node) -> None:
        self.ros_node = ros_node
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(855, 608)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(0, 0, 981, 600))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.tabWidget.setFont(font)
        self.tabWidget.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.tabWidget.setIconSize(QtCore.QSize(16, 16))
        self.tabWidget.setObjectName("tabWidget")
        self.tab_6 = QtWidgets.QWidget()
        self.tab_6.setObjectName("tab_6")
        self.MCMODE = QtWidgets.QPushButton(self.tab_6)
        self.MCMODE.setGeometry(QtCore.QRect(100, 100, 81, 31))
        self.MCMODE.clicked.connect(lambda: self.ros_node.modechange(1))
        self.MCGCLose = QtWidgets.QPushButton(self.tab_6)
        self.MCGCLose.setGeometry(QtCore.QRect(200, 480, 81, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.MCGCLose.setFont(font)
        self.MCGCLose.setObjectName("MCGCLose")
        self.label_46 = QtWidgets.QLabel(self.tab_6)
        self.label_46.setGeometry(QtCore.QRect(150, 430, 121, 40))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_46.setFont(font)
        self.label_46.setObjectName("label_46")
        self.MCGOpen = QtWidgets.QPushButton(self.tab_6)
        self.MCGOpen.setGeometry(QtCore.QRect(100, 480, 81, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.MCGOpen.setFont(font)
        self.MCGOpen.setObjectName("MCGOpen")
        self.comboBox = QtWidgets.QComboBox(self.tab_6)
        self.comboBox.setGeometry(QtCore.QRect(300, 140, 61, 22))
        font = QtGui.QFont()
        font.setPointSize(8)
        self.comboBox.setFont(font)
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.lineEdit_2 = QtWidgets.QLineEdit(self.tab_6)
        self.lineEdit_2.setGeometry(QtCore.QRect(220, 140, 61, 41))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.lineEdit = QtWidgets.QLineEdit(self.tab_6)
        self.lineEdit.setGeometry(QtCore.QRect(100, 140, 61, 41))
        self.lineEdit.setObjectName("lineEdit")
        self.label_5 = QtWidgets.QLabel(self.tab_6)
        self.label_5.setGeometry(QtCore.QRect(180, 140, 21, 40))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.MCSend = QtWidgets.QPushButton(self.tab_6)
        self.MCSend.setGeometry(QtCore.QRect(170, 190, 41, 41))
        self.MCSend.setObjectName("MCSend")
        self.MCSend.clicked.connect(lambda: self.ros_node.pickplace(self.lineEdit.text(),self.lineEdit_2.text(),self.comboBox.currentIndex()))
        self.graphicsView_6 = QtWidgets.QGraphicsView(self.tab_6)
        self.graphicsView_6.setGeometry(QtCore.QRect(400, 40, 361, 271))
        self.graphicsView_6.setObjectName("graphicsView_6")
        self.tabWidget.addTab(self.tab_6, "")
        self.tab_4 = QtWidgets.QWidget()
        self.tab_4.setObjectName("tab_4")
        self.JJWj1 = QtWidgets.QSpinBox(self.tab_4)
        self.JJWj1.setGeometry(QtCore.QRect(120, 80, 61, 31))
        self.JJWj1.setObjectName("JJWj1")
        self.JJVJ2 = QtWidgets.QSpinBox(self.tab_4)
        self.JJVJ2.setGeometry(QtCore.QRect(120, 130, 61, 31))
        self.JJVJ2.setObjectName("JJVJ2")
        self.JJWJ3 = QtWidgets.QSpinBox(self.tab_4)
        self.JJWJ3.setGeometry(QtCore.QRect(120, 180, 61, 31))
        self.JJWJ3.setObjectName("JJWJ3")
        self.JJWJ4 = QtWidgets.QSpinBox(self.tab_4)
        self.JJWJ4.setGeometry(QtCore.QRect(120, 230, 61, 31))
        self.JJWJ4.setObjectName("JJWJ4")
        self.label_12 = QtWidgets.QLabel(self.tab_4)
        self.label_12.setGeometry(QtCore.QRect(220, 70, 51, 41))
        self.label_12.setObjectName("label_12")
        self.label_13 = QtWidgets.QLabel(self.tab_4)
        self.label_13.setGeometry(QtCore.QRect(220, 120, 51, 41))
        self.label_13.setObjectName("label_13")
        self.label_14 = QtWidgets.QLabel(self.tab_4)
        self.label_14.setGeometry(QtCore.QRect(220, 170, 51, 41))
        self.label_14.setObjectName("label_14")
        self.label_15 = QtWidgets.QLabel(self.tab_4)
        self.label_15.setGeometry(QtCore.QRect(220, 220, 51, 41))
        self.label_15.setObjectName("label_15")
        self.graphicsView = QtWidgets.QGraphicsView(self.tab_4)
        self.graphicsView.setGeometry(QtCore.QRect(400, 40, 361, 271))
        self.graphicsView.setObjectName("graphicsView")
        self.JJSTART = QtWidgets.QPushButton(self.tab_4)
        self.JJSTART.setGeometry(QtCore.QRect(130, 310, 91, 31))
        self.JJSTART.setObjectName("JJSTART")
        self.JJSTOP = QtWidgets.QPushButton(self.tab_4)
        self.JJSTOP.setGeometry(QtCore.QRect(240, 310, 91, 31))
        self.JJSTOP.setObjectName("JJSTOP")
        self.JJMODE = QtWidgets.QPushButton(self.tab_4)
        self.JJMODE.setGeometry(QtCore.QRect(50, 50, 81, 31))
        self.JJMODE.clicked.connect(lambda: self.ros_node.modechange(2))
        self.tabWidget.addTab(self.tab_4, "")
        self.tab_7 = QtWidgets.QWidget()
        self.tab_7.setObjectName("tab_7")
        self.label_18 = QtWidgets.QLabel(self.tab_7)
        self.label_18.setGeometry(QtCore.QRect(220, 170, 51, 41))
        self.label_18.setObjectName("label_18")
        self.label_16 = QtWidgets.QLabel(self.tab_7)
        self.label_16.setGeometry(QtCore.QRect(220, 70, 51, 41))
        self.label_16.setObjectName("label_16")
        self.label_17 = QtWidgets.QLabel(self.tab_7)
        self.label_17.setGeometry(QtCore.QRect(220, 120, 51, 41))
        self.label_17.setObjectName("label_17")
        self.label_19 = QtWidgets.QLabel(self.tab_7)
        self.label_19.setGeometry(QtCore.QRect(220, 220, 61, 41))
        self.label_19.setObjectName("label_19")
        self.LJVx = QtWidgets.QSpinBox(self.tab_7)
        self.LJVx.setGeometry(QtCore.QRect(120, 80, 61, 31))
        self.LJVx.setObjectName("LJVx")
        self.LJVY = QtWidgets.QSpinBox(self.tab_7)
        self.LJVY.setGeometry(QtCore.QRect(120, 130, 61, 31))
        self.LJVY.setObjectName("LJVY")
        self.LJVZ = QtWidgets.QSpinBox(self.tab_7)
        self.LJVZ.setGeometry(QtCore.QRect(120, 180, 61, 31))
        self.LJVZ.setObjectName("LJVZ")
        self.LJWyaw = QtWidgets.QSpinBox(self.tab_7)
        self.LJWyaw.setGeometry(QtCore.QRect(120, 230, 61, 31))
        self.LJWyaw.setObjectName("LJWyaw")
        self.LJSTART = QtWidgets.QPushButton(self.tab_7)
        self.LJSTART.setGeometry(QtCore.QRect(130, 310, 91, 31))
        self.LJSTART.setObjectName("LJSTART")
        self.LJSTOP = QtWidgets.QPushButton(self.tab_7)
        self.LJSTOP.setGeometry(QtCore.QRect(240, 310, 91, 31))
        self.LJSTOP.setObjectName("JJSTOP")
        self.LJMODE = QtWidgets.QPushButton(self.tab_7)
        self.LJMODE.setGeometry(QtCore.QRect(50, 50, 81, 31))
        self.LJMODE.clicked.connect(lambda: self.ros_node.modechange(3))
        self.graphicsView_3 = QtWidgets.QGraphicsView(self.tab_7)
        self.graphicsView_3.setGeometry(QtCore.QRect(400, 40, 361, 271))
        self.graphicsView_3.setObjectName("graphicsView_3")
        self.tabWidget.addTab(self.tab_7, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.label_7 = QtWidgets.QLabel(self.tab_2)
        self.label_7.setGeometry(QtCore.QRect(200, 40, 151, 51))
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.Camerahere_2 = QtWidgets.QGraphicsView(self.tab_2)
        self.Camerahere_2.setGeometry(QtCore.QRect(70, 110, 401, 401))
        self.Camerahere_2.setObjectName("Camerahere_2")
        self.RECOGNIZE = QtWidgets.QPushButton(self.tab_2)
        self.RECOGNIZE.setGeometry(QtCore.QRect(540, 180, 151, 51))
        self.RECOGNIZE.setObjectName("RECOGNIZE")
        self.lineEdit_3 = QtWidgets.QLineEdit(self.tab_2)
        self.lineEdit_3.setGeometry(QtCore.QRect(540, 240, 151, 41))
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.pushButton = QtWidgets.QPushButton(self.tab_2)
        self.pushButton.setGeometry(QtCore.QRect(700, 240, 71, 41))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_2.setGeometry(QtCore.QRect(540, 300, 151, 41))
        self.pushButton_2.setObjectName("pushButton_2")
        # self.pushButton_2.clicked.connect(self.ros_node.sendfen)
        self.tabWidget.addTab(self.tab_2, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(5)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        ##################################################################################################
        self.message_queue = collections.deque()
        self.worker = Ui_MainWindow.Worker(self.message_queue,self.ros_node)
        self.thread1 = QtCore.QThread()
        self.worker.moveToThread(self.thread1)
        self.thread1.started.connect(lambda: self.worker.process_messages(self.JJWj1.value(),self.JJVJ2.value(),self.JJWJ3.value(),self.JJWJ4.value()))
        self.JJSTART.clicked.connect(self.enqueue_message)
        self.JJSTOP.clicked.connect(self.closeEvent)

        self.worker2 = Ui_MainWindow.Worker2(self.message_queue,self.ros_node)
        self.thread2 = QtCore.QThread()
        self.worker2.moveToThread(self.thread2)
        self.thread2.started.connect(lambda: self.worker2.process_messages2(self.LJVx.value(),self.LJVY.value(),self.LJVZ.value(),self.LJWyaw.value()))
        self.LJSTART.clicked.connect(self.enqueue_message2)
        self.LJSTOP.clicked.connect(self.closeEvent2)


    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.MCGCLose.setText(_translate("MainWindow", "Close"))
        self.label_46.setText(_translate("MainWindow", "Gripper"))
        self.MCGOpen.setText(_translate("MainWindow", "Open"))        
        self.comboBox.setItemText(0, _translate("MainWindow", "Move"))
        self.comboBox.setItemText(1, _translate("MainWindow", "Capture"))
        self.comboBox.setItemText(2, _translate("MainWindow", "Promote"))
        self.comboBox.setItemText(3, _translate("MainWindow", "Castling"))
        self.label_5.setText(_translate("MainWindow", "To"))
        self.MCSend.setText(_translate("MainWindow", "GO!"))
        self.MCMODE.setText(_translate("MainWindow", "mode"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_6), _translate("MainWindow", "Manual Chess"))
        self.label_12.setText(_translate("MainWindow", "Wj1"))
        self.label_13.setText(_translate("MainWindow", "Vj2"))
        self.label_14.setText(_translate("MainWindow", "Wj3"))
        self.label_15.setText(_translate("MainWindow", "Wj4"))
        self.JJSTART.setText(_translate("MainWindow", "START"))
        self.JJSTOP.setText(_translate("MainWindow", "STOP"))
        self.JJMODE.setText(_translate("MainWindow", "mode"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_4), _translate("MainWindow", "Joint Jog"))
        self.label_18.setText(_translate("MainWindow", "Vz"))
        self.label_16.setText(_translate("MainWindow", "Vx"))
        self.label_17.setText(_translate("MainWindow", "Vy"))
        self.label_19.setText(_translate("MainWindow", "Wyaw"))
        self.LJSTART.setText(_translate("MainWindow", "START"))
        self.LJSTOP.setText(_translate("MainWindow", "STOP"))
        self.LJMODE.setText(_translate("MainWindow", "mode"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_7), _translate("MainWindow", "Linear Jog"))
        self.label_7.setText(_translate("MainWindow", "Game State"))
        self.RECOGNIZE.setText(_translate("MainWindow", "Recognize!"))
        self.pushButton.setText(_translate("MainWindow", "reload"))
        self.pushButton_2.setText(_translate("MainWindow", "Chess AI"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "Game State Recognition"))
    
    # @QtCore.pyqtSlot(bool)
    def enqueue_message(self):
        self.thread1.start()
        
    def closeEvent(self,e):
        self.thread1.requestInterruption()
        self.thread1.quit()
        self.thread1.wait()

    def enqueue_message2(self):
        self.thread2.start()
        
    def closeEvent2(self,e):
        self.thread2.requestInterruption()
        self.thread2.quit()
        self.thread2.wait()
        
    class Worker(QtCore.QObject):
        def __init__(self, message_queue,ros_node) -> None:
            super().__init__()
            self.ros_node = ros_node
            self.message_queue = message_queue
            
        @QtCore.pyqtSlot()    
        def process_messages(self):
            while not self.thread().isInterruptionRequested():
                time.sleep(0.1)
                self.ros_node.sendJJ()
                print(f"1")
            print("stop")

    class Worker2(QtCore.QObject):
        def __init__(self, message_queue,ros_node):
            super().__init__()
            self.ros_node = ros_node
            self.message_queue = message_queue

        @QtCore.pyqtSlot()    
        def process_messages2(self):
            while not self.thread().isInterruptionRequested():
                time.sleep(0.1)
                print(f"2")
            print("stop")
def main():
    rclpy.init(args=None)
    ros_node=GuiNode()
    executor = MultiThreadedExecutor()        
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow(ros_node)
    ui.setupUi(MainWindow)
    torarion=0
    MainWindow.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()


