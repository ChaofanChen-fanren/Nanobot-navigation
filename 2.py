from PyQt5 import QtCore, QtGui, QtWidgets


class MouseTracker(QtCore.QObject):
    positionChanged = QtCore.pyqtSignal(QtCore.QPoint)

    def __init__(self, widget):
        super().__init__(widget)
        self._widget = widget
        self.widget.setMouseTracking(True)
        self.widget.installEventFilter(self)

    @property
    def widget(self):
        return self._widget

    def eventFilter(self, o, e):
        if o is self.widget and e.type() == QtCore.QEvent.MouseMove:
            self.positionChanged.emit(e.pos())
        return super().eventFilter(o, e)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        self.video_label = QtWidgets.QLabel()
        self.video_label.setStyleSheet("background-color: green; border: 1px solid black")

        tracker = MouseTracker(self.video_label)
        tracker.positionChanged.connect(self.on_positionChanged)

        lay = QtWidgets.QVBoxLayout(central_widget)
        lay.addWidget(self.video_label)
        lay.addWidget(QtWidgets.QLabel())

        self.resize(640, 480)

        self.label_position = QtWidgets.QLabel(
            self.video_label, alignment=QtCore.Qt.AlignCenter
        )
        self.label_position.setStyleSheet('background-color: white; border: 1px solid black')

    @QtCore.pyqtSlot(QtCore.QPoint)
    def on_positionChanged(self, pos):
        delta = QtCore.QPoint(30, -15)
        self.label_position.show()
        self.label_position.move(pos + delta)
        self.label_position.setText("(%d, %d)" % (pos.x(), pos.y()))
        self.label_position.adjustSize()


if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())