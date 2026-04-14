import serial

ser = serial.Serial('COM4')

def read_exactly(ser, size):
    """Read exactly 'size' bytes, retrying on partial reads."""
    data = b''
    while len(data) < size:
        chunk = ser.read(size - len(data))
        if not chunk:
            break
        data += chunk
    return data

data = read_exactly(ser, 500)
with open('data.bin', 'wb') as fp:
    fp.write(data)
print(data)
print(f"Complete: {len(data) == 500}")