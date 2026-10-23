import socket
import struct
import time

# 对应 Amesim 配置:
# 监听 2001 端口 (接收 Amesim 发出的数据)
# 发往 2000 端口 (发送数据给 Amesim)
LOCAL_PORT = 2001
AMESIM_IP = '127.0.0.1'
AMESIM_PORT = 2000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', LOCAL_PORT))
sock.settimeout(0.02)  # 20ms 超时

print("=====================================================")
print(f" UDP 极简测试服务端已启动")
print(f" 本地监听端口: {LOCAL_PORT} | 目标 Amesim 端口: {AMESIM_PORT}")
print(" 规则: 收到 Amesim 的数值 -> 乘以 2 倍 -> 立即返回给 Amesim")
print("=====================================================")

try:
    while True:
        try:
            # 1. 接收来自 Amesim 的 1 个单精度浮点数 (float 4 字节)
            data, addr = sock.recvfrom(1024)
            if len(data) == 4:
                val_in = struct.unpack('<f', data)[0]
                
                # 2. 对数据进行简单加工 (例如放大 2 倍) 作为回传响应
                val_out = val_in * 2.0
                
                print(f"[收到 Amesim]: {val_in:.4f}  --->  [返回 Amesim]: {val_out:.4f}")
                
                # 3. 发回给 Amesim (4 字节)
                send_bytes = struct.pack('<f', val_out)
                sock.sendto(send_bytes, (AMESIM_IP, AMESIM_PORT))
        except socket.timeout:
            pass

        time.sleep(0.005)

except KeyboardInterrupt:
    print("\n>>> 服务已停止。")
    sock.close()
