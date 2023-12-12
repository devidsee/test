import modbus_tk.modbus_tcp as mt
import modbus_tk.defines as cst

class modbustool():
    def __init__(self,ip,port):
        self.master = mt.TcpMaster(ip, 502)
        self.master.set_timeout(5)
    #@params slave tongdao 0
    #@params v 0-4095
    def wright_0t5v(self,slave,v):
        #
        # '''
        # 寄存器类型：保持寄存器
        # 访问类型：读写
        # 功能码：0x03、0x06、0x10
        # '''
        # 0x06功能码：写单个保持寄存器
        single_register_value = self.master.execute(slave=slave, function_code=cst.WRITE_SINGLE_REGISTER, starting_address=0,
                                               output_value=v)
        print('0x06 WRITE_SINGLE_REGISTER: ', single_register_value)
    #@params slave tongdao 2
    #@params v 0-4095
    def wright_0t10v(self,slave,v):
        #
        # '''
        # 寄存器类型：保持寄存器
        # 访问类型：读写
        # 功能码：0x03、0x06、0x10
        # '''
        # 0x06功能码：写单个保持寄存器
        single_register_value = self.master.execute(slave=slave, function_code=cst.WRITE_SINGLE_REGISTER, starting_address=0,
                                               output_value=v)
        print('0x06 WRITE_SINGLE_REGISTER: ', single_register_value)
    #@params slave tongdao 3
    #@params v 0-4095
    def wright_4t20ma(self,slave,v):
        #
        # '''
        # 寄存器类型：保持寄存器
        # 访问类型：读写
        # 功能码：0x03、0x06、0x10
        # '''
        # 0x06功能码：写单个保持寄存器
        single_register_value = self.master.execute(slave=slave, function_code=cst.WRITE_SINGLE_REGISTER, starting_address=0,
                                               output_value=v)
        print('0x06 WRITE_SINGLE_REGISTER: ', single_register_value)
    #@params slave tongdao 16
    #@params v 0/1
    def wright_0or1(self,slave,v):
        #
        # '''
        # 寄存器类型：保持寄存器
        # 访问类型：读写
        # 功能码：0x03、0x06、0x10
        # '''
        # 0x06功能码：写单个保持寄存器
        single_coil_value = self.master.execute(slave=1, function_code=cst.WRITE_SINGLE_COIL, starting_address=17,
                                           output_value=1)  # 写单个线圈状态为 ON
        print('0x05 WRITE_SIGNLE_COIL: ', single_coil_value)
        # # 数据类型
        # # 写单个寄存器：无符号整数
        single_register_value = self.master.execute(slave=1, function_code=cst.WRITE_SINGLE_REGISTER, starting_address=16,
                                               output_value=1)
        print('0x06 WRITE_SINGLE_REGISTER: ', single_register_value)

