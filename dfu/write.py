import hid
from intelhex import IntelHex
import time

# 设备VID和PID，请替换成你实际设备的VID和PID
VENDOR_ID = 0x1A86  # 替换为你的设备VID
PRODUCT_ID = 0x0002  # 替换为你的设备PID

# HEX文件路径
HEX_FILE_PATH = "C:/Users/Kawai/Desktop/HSDAC/HSDAC/obj/HSDAC.hex"  # 替换为你的hex文件路径

# HID报告的最大长度，根据你的设备进行调整
REPORT_SIZE = 256  # 常见的HID报告大小是64字节

def burn_hex_file_to_hid(vendor_id, product_id, hex_file_path, report_size):
    """
    将HEX文件烧写到HID设备。

    Args:
        vendor_id (int): 设备的供应商ID。
        product_id (int): 设备的產品ID。
        hex_file_path (str): HEX文件的路径。
        report_size (int): HID报告的大小。
    """
    try:
        # 打开HID设备
        h = hid.device()
        h.open(vendor_id, product_id)
        h.set_nonblocking(False)

        print(f"成功连接到设备 VID: {vendor_id}, PID: {product_id}")

        # 读取HEX文件
        ih = IntelHex(hex_file_path)
        # 将HEX文件转换为二进制数据
        binary_data = ih.tobinarray()

        # 将数据分割成报告大小的块
        chunks = [binary_data[i:i + report_size] for i in range(0, len(binary_data), report_size)]

        # 发送数据块
        for chunk in chunks:
            # 填充数据块，使其达到报告大小
            padding_size = report_size - len(chunk)
            padded_chunk = list(chunk) + [0x00] * padding_size  # 使用0x00进行填充

            # 将数据块转换为列表, 0x00是数据块
            report = [0x00] + [0x00, 0x00, 0x00, 0x00] + padded_chunk  # 添加报告ID (这里使用0x00)

            # 发送报告
            h.write(report)
            print(f"发送了: {len(chunk)} 字节")
            time.sleep(0.1)  # 添加一个小的延迟
        # 发送结束块
        h.write([0x00, 0xff, 0xff, 0xff, 0xff] + [0x00] * report_size)
        time.sleep(10)

        # 重新发送BIN校验数据
        print("开始校验数据")
        for chunk in chunks:
            # 填充数据块，使其达到报告大小
            padding_size = report_size - len(chunk)
            padded_chunk = list(chunk) + [0x00] * padding_size  # 使用0x00进行填充

            # 将数据块转换为列表, 0x00是数据块
            report = [0x00] + [0x00, 0x00, 0x00, 0x00] + padded_chunk  # 添加报告ID (这里使用0x00)

            # 发送报告
            h.write(report)
            print(f"校验了 {len(chunk)} 字节")
            time.sleep(0.1)  # 添加一个小的延迟
        # 发送结束块
        h.write([0x00, 0xff, 0xff, 0xff, 0xff] + [0x00] * report_size)
        time.sleep(10)

        print("文件烧写完成")

    except IOError as ex:
        print(ex)
        print("无法找到设备或发生其他I/O错误。请检查设备是否已连接，VID和PID是否正确。")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        if 'h' in locals() and h:
            h.close()
            print("断开连接")

if __name__ == "__main__":
    burn_hex_file_to_hid(VENDOR_ID, PRODUCT_ID, HEX_FILE_PATH, REPORT_SIZE)