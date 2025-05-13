import serial

SERIAL_PORT = 'COM8'  # Change selon ton port
BAUDRATE = 115200     # Peu importe ici, c’est USB, mais ça évite des bugs parfois

try:
    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
        print(f"Écoute sur {SERIAL_PORT}...")
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                print(f"[STM32] {line}")
except serial.SerialException as e:
    print(f"Erreur : {e}")