# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'betterVSlider_grid.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(197, 138)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(Form)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.label_variable = QtWidgets.QLabel(Form)
        self.label_variable.setObjectName("label_variable")
        self.gridLayout.addWidget(self.label_variable, 0, 0, 1, 1)
        self.doubleSpinBox = QtWidgets.QDoubleSpinBox(Form)
        self.doubleSpinBox.setObjectName("doubleSpinBox")
        self.gridLayout.addWidget(self.doubleSpinBox, 0, 1, 1, 1)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_max = QtWidgets.QLabel(Form)
        self.label_max.setObjectName("label_max")
        self.verticalLayout.addWidget(self.label_max, 0, QtCore.Qt.AlignRight|QtCore.Qt.AlignTop)
        self.label_min = QtWidgets.QLabel(Form)
        self.label_min.setObjectName("label_min")
        self.verticalLayout.addWidget(self.label_min, 0, QtCore.Qt.AlignRight|QtCore.Qt.AlignBottom)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.slider = QtWidgets.QSlider(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.slider.sizePolicy().hasHeightForWidth())
        self.slider.setSizePolicy(sizePolicy)
        self.slider.setOrientation(QtCore.Qt.Vertical)
        self.slider.setTickPosition(QtWidgets.QSlider.TicksBothSides)
        self.slider.setObjectName("slider")
        self.horizontalLayout.addWidget(self.slider, 0, QtCore.Qt.AlignLeft)
        self.gridLayout.addLayout(self.horizontalLayout, 1, 1, 1, 1)
        self.verticalLayout_2.addLayout(self.gridLayout)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label_variable.setText(_translate("Form", "TextLabel"))
        self.label_max.setText(_translate("Form", "Max value"))
        self.label_min.setText(_translate("Form", "Min value"))