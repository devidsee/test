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

# if __name__ == '__main__':
    # master = mt.TcpMaster('169.254.94.196', 502)
    # master.set_timeout(5)
    # # 参数说明
    # # slave: Modbus从站地址. from 1 to 247. 0为广播所有的slave tongdao
    # # function_code：功能码
    # # starting_address：寄存器起始地址
    # # quantity_of_x：寄存器读写的数量，写寄存器时数量可为 0，读寄存器时数量至少为 1; 一个寄存器=2字节, 1字节=8位
    # # output_value：输出内容，读操作无效，写操作是一个整数或可迭代的list值：1 / [1,1,1,0,0,1] / xrange(12)
    # # data_format：对数据进行格式化 >表示大端模式, <表示小端模式, H表示unsigned short无符号整形(2字节), h表示short有符号整型(2字节),
    # # l表示long长整型(4字节), f表示float浮点型(4字节), d表示double双精度浮点型(8字节)
    # # expected_length
    #
    # try:
    #     '''
    #     寄存器类型：线圈状态
    #     访问类型：读写
    #     功能码：0x01、0x05、0x0F
    #     '''
    #     '''
    #     0x05功能码：写线圈状态为 开ON(0xff00)/关OFF(0), output_value不为0时都会置为0xff00
    #     ON的output_value可以设置为 0xff00、True、非0值； OFF的output_value 可以设置为 0x0000、False、0
    #     返回结果：tuple(地址, 值) ，写成功：如写入开则返回0xff00的十进制格式65280，写入关则返回0x0000的十进制格式0
    #     '''
    #     ''' 0x05功能码: 写单个线圈状态 ON '''
    #     # single_coil_value = master.execute(slave=1, function_code=cst.WRITE_SINGLE_COIL, starting_address=0, output_value=1)     # 写单个线圈状态为 ON
    #     # print('0x05 WRITE_SIGNLE_COIL: ', single_coil_value)
    #     #
    #     # ''' 0x01功能码：读线圈状态 '''
    #     coils_value = master.execute(slave=1, function_code=cst.READ_COILS, starting_address=0, quantity_of_x=1)    # 读线圈状态
    #     print('0x01 READ_COILS: ', coils_value)
    #
    #     # ''' 0x05功能码: 写单个线圈状态 OFF '''
    #     # single_coil_value = master.execute(slave=1, function_code=cst.WRITE_SINGLE_COIL, starting_address=1, output_value=0)    # 写单个线圈状态为 OFF
    #     # print('0x05 WRITE_SIGNLE_COIL: ', single_coil_value)
    #     coils_value = master.execute(slave=1, function_code=cst.READ_COILS, starting_address=1, quantity_of_x=1)    # 读线圈状态
    #     print('0x01 READ_COILS: ', coils_value)
    #     # '''
    #     # 寄存器类型：输入寄存器
    #     # 访问类型：只读
    #     # 功能码：0x04
    #     # '''
    #     # # 0x04功能码：读输入寄存器
    #     input_value = master.execute(slave=1, function_code=cst.READ_INPUT_REGISTERS, starting_address=0, quantity_of_x=5)
    #     print('0x04 READ_INPUT_REGISTERS: ', input_value)
    #     #
    #     # '''
    #     # 寄存器类型：保持寄存器
    #     # 访问类型：读写
    #     # 功能码：0x03、0x06、0x10
    #     # '''
    #     # 0x06功能码：写单个保持寄存器
    #     single_register_value = master.execute(slave=1, function_code=cst.WRITE_SINGLE_REGISTER, starting_address=0, output_value=0)
    #     print('0x06 WRITE_SINGLE_REGISTER: ', single_register_value)
    #
    # except Exception as e:
    #     print('error: %s' % e)

