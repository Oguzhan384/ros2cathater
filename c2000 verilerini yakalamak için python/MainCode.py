import serial
import struct
import time
import csv
import os
from datetime import datetime

# --- AYARLAR ---
COM_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200
V_REF = 3.0          
OFFSET = 1.5         

# --- DOSYA YOLU AYARI ---
# Dosyayı bulması en kolay yer olan Masaüstü'ne kaydeder
masaustu = os.path.join(os.path.expanduser("~"), "Desktop")
# Eğer sisteminiz Türkçe ise ve masaüstü yolu bulunamazsa kodun olduğu yere kaydeder
if not os.path.exists(masaustu):
    masaustu = os.path.dirname(os.path.abspath(__file__))

dosya_adi = datetime.now().strftime("Veri_Kaydi_%Y%m%d_%H%M%S.csv")
tam_yol = os.path.join(masaustu, dosya_adi)

# --- DEĞİŞKENLER ---
ilk_encoder_degeri = None  # Sıfırlama için ilk değeri tutacak

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"--- {COM_PORT} Bağlantısı Açıldı ---")
    print(f"📂 Kayıt Dosyası: {tam_yol}")
    print("Durdurmak için Ctrl+C'ye basın...")

    with open(tam_yol, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Zaman (sn)', 'Voltaj (V)', 'Encoder']) 

        start_time = time.time()

        while True:
            if ser.in_waiting >= 6:
                raw = ser.read(6)
                
                # 1. ADC -> Voltaj Hesapla
                adc_val = (raw[0] << 8) | raw[1]
                voltage = (adc_val * V_REF / 4095.0) - OFFSET
                
                # 2. Encoder Çöz ve Sıfırla
                ham_encoder = struct.unpack('>i', raw[2:6])[0]
                
                if ilk_encoder_degeri is None:
                    ilk_encoder_degeri = ham_encoder # İlk gelen veriyi 'sıfır' noktası kabul et
                
                sifirlanmis_encoder = ham_encoder - ilk_encoder_degeri
                
                # 3. Zaman
                elapsed_time = round(time.time() - start_time, 3)
                
                # Ekrana Yazdır
                print(f"T: {elapsed_time} | V: {voltage:6.3f} | Encoder: {sifirlanmis_encoder}")
                
                # Dosyaya Yaz
                writer.writerow([elapsed_time, round(voltage, 4), sifirlanmis_encoder])
                f.flush() 
            else:
                time.sleep(0.005)

except KeyboardInterrupt:
    print(f"\n\n✅ Kayıt tamamlandı. Dosyanız Masaüstü'nde:\n📍 {tam_yol}")

except Exception as e:
    print(f"\n❌ Hata: {e}")

finally:
    if 'ser' in locals(): 
        ser.close()