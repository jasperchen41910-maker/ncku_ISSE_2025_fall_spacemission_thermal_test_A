import serial
import time
import os
from datetime import datetime

# 設定區
COM_PORT = 'COM7'
BAUD_RATES = 9600

# --- 1. 準備檔案與路徑 ---
if not os.path.exists('logs'):
    os.makedirs('logs')

# 取得當前時間字串
time_str =   datetime.now().strftime("%Y%m%d_%H%M%S")

# 定義兩個不同的檔名
file_path_2s = f"logs/Packet_2s_{time_str}.csv"
file_path_10s = f"logs/Packet_10s_{time_str}.csv"

print(f"正在開啟 {COM_PORT}...")
print(f"2s 封包將存入: {file_path_2s}")
print(f"10s 封包將存入: {file_path_10s}")

try:
    ser = serial.Serial(COM_PORT, BAUD_RATES, timeout=1)

    # --- 2. 同時開啟兩個檔案 ---
    # 我們使用 'w' 模式開啟，並分別給予不同的變數名稱 (f_a, f_b)
    f_2 = open(file_path_2s, 'w', encoding='utf-8')
    f_10 = open(file_path_10s, 'w', encoding='utf-8')

    # (選用) 先寫入 CSV 標題欄位，方便 Excel 讀取
    # 這裡我多加了一個 "PC_Time" 欄位，記錄電腦收到的時間
    f_2.write("PC_Time,GPS_time,Lat,Lon,Alt,Sat,Speed,Pitch,Roll,Yaw,q0,q1,q2,q3,bx,by,bz,,timer,Temp\n")
    f_10.write("PC_Time,Time,Alt,CPM,uSvh,nSvh,counts,ADC0,ADC3,ADC4,ADC5,ADC6,ADCV\n")

    print("接收端啟動成功！開始分流存檔...")

    while True:
        if ser.in_waiting:
            try:
                # 讀取並解碼
                data_raw = ser.readline()
                data_str = data_raw.decode('utf-8', errors='ignore').strip()

                if not data_str: continue

                # 取得電腦當下時間 (方便對照)
                pc_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]

                # --- 3. 解析與分流 ---
                values = data_str.split(',')
                header = values[0]
                content_to_save = f"{pc_time},{data_str[2:]}\n"

                if header == "Packet2s":
                    # 寫入 2s 檔案
                    f_2.write(content_to_save)
                    f_2.flush()  # 立即存檔
                    print(f"{data_str}")

                elif header == "Packet10s":
                    # 寫入 10s 檔案
                    f_10.write(content_to_save)
                    f_10.flush()  # 立即存檔
                    print(f"{data_str}")

                else:
                    print(f"{data_str}")

            except Exception as e:
                print(f"{e}")

        time.sleep(0.01)

except KeyboardInterrupt:
    # 程式結束時，記得關閉兩個檔案
    if 'f_2s' in locals(): f_2.close()
    if 'f_10s' in locals(): f_10.close()
    if 'ser' in locals() and ser.is_open: ser.close()
    print('\n程式已停止，檔案已關閉。')
except serial.SerialException as e:
    print(f"\n無法開啟 COM Port: {e}")
