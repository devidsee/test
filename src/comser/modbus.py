import modbus_tk.modbus_tcp as mt
import modbus_tk.defines as cst


if __name__ == '__main__':
    master = mt.TcpMaster('192.168.0.110', 502)
    master.set_timeout(5)
    # 参数说明
    # slave: Modbus从站地址. from 1 to 247. 0为广播所有的slave
    # function_code：功能码
    # starting_address：寄存器起始地址
    # quantity_of_x：寄存器读写的数量，写寄存器时数量可为 0，读寄存器时数量至少为 1; 一个寄存器=2字节, 1字节=8位
    # output_value：输出内容，读操作无效，写操作是一个整数或可迭代的list值：1 / [1,1,1,0,0,1] / xrange(12)
    # data_format：对数据进行格式化 >表示大端模式, <表示小端模式, H表示unsigned short无符号整形(2字节), h表示short有符号整型(2字节), l表示long长整型(4字节), f表示float浮点型(4字节), d表示double双精度浮点型(8字节)
    # expected_length

    try:
        '''
        寄存器类型：线圈状态
        访问类型：读写
        功能码：0x01、0x05、0x0F
        '''
        '''
        0x05功能码：写线圈状态为 开ON(0xff00)/关OFF(0), output_value不为0时都会置为0xff00
        ON的output_value可以设置为 0xff00、True、非0值； OFF的output_value 可以设置为 0x0000、False、0
        返回结果：tuple(地址, 值) ，写成功：如写入开则返回0xff00的十进制格式65280，写入关则返回0x0000的十进制格式0
        '''
        ''' 0x05功能码: 写单个线圈状态 ON '''
        single_coil_value = master.execute(slave=1, function_code=cst.WRITE_SINGLE_COIL, starting_address=0, output_value=1)     # 写单个线圈状态为 ON
        print('0x05 WRITE_SIGNLE_COIL: ', single_coil_value)

        ''' 0x01功能码：读线圈状态 '''
        coils_value = master.execute(slave=1, function_code=cst.READ_COILS, starting_address=0, quantity_of_x=1)    # 读线圈状态
        print('0x01 READ_COILS: ', coils_value)

        ''' 0x05功能码: 写单个线圈状态 OFF '''
        single_coil_value = master.execute(slave=1, function_code=cst.WRITE_SINGLE_COIL, starting_address=1, output_value=0)    # 写单个线圈状态为 OFF
        print('0x05 WRITE_SIGNLE_COIL: ', single_coil_value)
        coils_value = master.execute(slave=1, function_code=cst.READ_COILS, starting_address=1, quantity_of_x=1)    # 读线圈状态
        print('0x01 READ_COILS: ', coils_value)

        ''' 0x0F功能码：写多个线圈状态 '''
        multiple_coils_value = master.execute(slave=1, function_code=cst.WRITE_MULTIPLE_COILS, starting_address=2, quantity_of_x=4, output_value=[1, 1, 1, 1])  # 写多个线圈
        print('0x0F WRITE_COILS_REGISTER: ', multiple_coils_value)
        coils_value = master.execute(slave=1, function_code=cst.READ_COILS, starting_address=2, quantity_of_x=4)    # 读线圈状态
        print('0x01 READ_COILS: ', coils_value)

        '''
        寄存器类型：离散输入状态
        访问类型：只读
        功能码：0x02
        '''
        # 0x02功能码：读离散输入状态
        discrete_value = master.execute(slave=1, function_code=cst.READ_DISCRETE_INPUTS, starting_address=0, quantity_of_x=5)
        print('0x02 READ_DISCRETE_INPUTS: ', discrete_value)

        '''
        寄存器类型：输入寄存器
        访问类型：只读
        功能码：0x04
        '''
        # 0x04功能码：读输入寄存器
        input_value = master.execute(slave=1, function_code=cst.READ_INPUT_REGISTERS, starting_address=0, quantity_of_x=5)
        print('0x04 READ_INPUT_REGISTERS: ', input_value)

        '''
        寄存器类型：保持寄存器
        访问类型：读写
        功能码：0x03、0x06、0x10
        '''
        # 0x06功能码：写单个保持寄存器
        single_register_value = master.execute(slave=1, function_code=cst.WRITE_SINGLE_REGISTER, starting_address=0, output_value=666)
        print('0x06 WRITE_SINGLE_REGISTER: ', single_register_value)

        # 0x03功能码：读保持寄存器
        holding_value = master.execute(slave=1, function_code=cst.READ_HOLDING_REGISTERS, starting_address=0, quantity_of_x=1)
        print('0x03 READ_HOLDING_REGISTERS: ', holding_value)

        # 0x10功能码：写多个保持寄存器
        multiple_registers_value = master.execute(slave=1, function_code=cst.WRITE_MULTIPLE_REGISTERS, starting_address=1, quantity_of_x=3, output_value=[777, 777, 777])
        print('0x10 WRITE_MULTIPLE_REGISTERS: ', multiple_registers_value)
        holding_value = master.execute(slave=1, function_code=cst.READ_HOLDING_REGISTERS, starting_address=1, quantity_of_x=3)
        print('0x03 READ_HOLDING_REGISTERS: ', holding_value)


        # 数据类型
        # 写单个寄存器：无符号整数
        single_register_value = master.execute(slave=1, function_code=cst.WRITE_SINGLE_REGISTER, starting_address=0, output_value=4097)
        print('0x06 WRITE_SINGLE_REGISTER: ', single_register_value)
        # 写单个寄存器：有符号整数
        single_register_value = master.execute(slave=1, function_code=cst.WRITE_SINGLE_REGISTER, starting_address=1, output_value=-1234)
        print('0x06 WRITE_SINGLE_REGISTER: ', single_register_value)
        # 写多个寄存器：有符号整数 （根据列表长度来判读写入个数）
        multiple_registers_value = master.execute(slave=1, function_code=cst.WRITE_MULTIPLE_REGISTERS, starting_address=2, output_value=[1, -2], data_format='>hh')
        print('0x10 WRITE_MULTIPLE_REGISTERS: ', multiple_registers_value)
        # 读寄存器
        holding_value = master.execute(slave=1, function_code=cst.READ_HOLDING_REGISTERS, starting_address=0, quantity_of_x=4, data_format='>hhhh')
        print('0x03 READ_HOLDING_REGISTERS: ', holding_value)

        # 写多个寄存器: 浮点数float（float长度为4个字节，占用2个寄存器）
        # 起始地址为8的保持寄存器，操作寄存器个数为 4 ，一个浮点数float 占两个寄存器;
        # 写浮点数时一定要加 data_format 参数，两个ff 表示要写入两个浮点数，以此类推
        # 我这里模拟的是大端模式，具体可参考 struct 用法。和数据源保持一致即可。 <表示小端，>表示大端
        multiple_registers_value = master.execute(slave=1, function_code=cst.WRITE_MULTIPLE_REGISTERS, starting_address=8, output_value=[1.0, -6.4], data_format='>ff')
        print('0x10 WRITE_MULTIPLE_REGISTERS: ', multiple_registers_value)
        # 读对应的 4个寄存器（2个float），指定数据格式
        holding_value = master.execute(slave=1, function_code=cst.READ_HOLDING_REGISTERS, starting_address=8, quantity_of_x=4, data_format='>ff')
        print('0x03 READ_HOLDING_REGISTERS: ', holding_value)

        # 写多个寄存器：长整型数据long（long长度为4字节，占用2个寄存器）
        multiple_registers_value = master.execute(slave=1, function_code=cst.WRITE_MULTIPLE_REGISTERS, starting_address=12, output_value=[111111, -222222], data_format='>ll')
        print('0x10 WRITE_MULTIPLE_REGISTERS: ', multiple_registers_value)
        # 读对应的 4个寄存器（2个double），指定数据格式
        holding_value = master.execute(slave=1, function_code=cst.READ_HOLDING_REGISTERS, starting_address=12, quantity_of_x=4, data_format='>ll')
        print('0x03 READ_HOLDING_REGISTERS: ', holding_value)

        # 写多个寄存器：双精度浮点数double（double长度为8个字节，占用4个寄存器）
        multiple_registers_value = master.execute(slave=1, function_code=cst.WRITE_MULTIPLE_REGISTERS, starting_address=16, output_value=[1, -6.4], data_format='>dd')
        print('0x10 WRITE_MULTIPLE_REGISTERS: ', multiple_registers_value)
        # 读对应的 4个寄存器（2个double），指定数据格式
        holding_value = master.execute(slave=1, function_code=cst.READ_HOLDING_REGISTERS, starting_address=16, quantity_of_x=8, data_format='>dd')
        print('0x03 READ_HOLDING_REGISTERS: ', holding_value)
    except Exception as e:
        print('error: %s' % e)

