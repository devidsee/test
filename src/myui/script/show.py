# 导入程序运行必须模块
import sys
# PyQt5中使用的基本控件都在PyQt5.QtWidgets模块中
from PyQt5.QtWidgets import QApplication, QMainWindow
# 导入designer工具生成的login模块
from myui import Ui_MainWindow
import rospy
from std_msgs.msg import String
import roslib
from ..comser.modbustools import  modbustool
class MyMainForm(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MyMainForm, self).__init__(parent)
        self.setupUi(self)

        rospy.init_node("control_root",anonymous=True)
        self.pub = rospy.Publisher("base_msg",String,queue_size = 10)

        self.layer_sb.setRange(0, 100)  # 设置取值范围(最大值, 最小值)
        self.layer_sb.setValue(50)  # 设置当前值
        self.layer_sb.setSingleStep(1)  # 步长，每按一下按钮改变的值
        self.layer_pro.setRange(0, 100)  # 设置取值范围(最大值, 最小值)
        self.layer_pro.setValue(50)  # 设置当前值.

        self.push_sb.setRange(0, 100)  # 设置取值范围(最大值, 最小值)
        self.push_sb.setValue(50)  # 设置当前值
        self.push_sb.setSingleStep(1)  # 步长，每按一下按钮改变的值
        self.push_pro.setRange(0, 100)  # 设置取值范围(最大值, 最小值)
        self.push_pro.setValue(50)  # 设置当前值

        self.layer_sb.valueChanged.connect(self.layersb2pb)  # 值改变时
        self.push_sb.valueChanged.connect(self.pushsb2pb)  # 值改变时

    def layersb2pb(self):
        self.layer_pro.setValue(self.layer_sb.value())  # value()获得当前值

    def pushsb2pb(self):
        self.push_pro.setValue(self.push_sb.value())  # value()获得当前值
    def connect(self):
        self.modi6217 = modbustool(self.ip_i6217.getValue(),502)
        self.modo6227 = modbustool(self.ip_o6224.getValue(),502)
        self.modio_ip1 = modbustool(self.io_ip1.getValue(),502)
        self.modio_ip2 = modbustool(self.io_ip2.getValue(),502) 
        self.modio_ip3 = modbustool(self.io_ip3.getValue(),502)
        self.modio_ip4 = modbustool(self.io_ip4.getValue(),502)


if __name__ == "__main__":
    # 固定的，PyQt5程序都需要QApplication对象。sys.argv是命令行参数列表，确保程序可以双击运行
    app = QApplication(sys.argv)
    # 初始化
    myWin = MyMainForm()
    # 将窗口控件显示在屏幕上
    myWin.show()
    # 程序运行，sys.exit方法确保程序完整退出。
    sys.exit(app.exec_())
