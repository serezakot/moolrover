import time 
import serial

try:
    ser = serial.Serial(                # забираем порт у системы 
        port="/dev/ttyAMA0",
        baudrate=9600,                  #скорочть передачи
        parity=serial.PARITY_NONE,      # проверка 
        timeout= 1,                     # Set a read timeout value in seconds.
        bytesize=serial.EIGHTBITS,      #Number of data bits. Possible values: FIVEBITS, SIXBITS, SEVENBITS, EIGHTBITS
        stopbits = serial.STOPBITS_ONE, #Number of stop bits. Possible values: STOPBITS_ONE, STOPBITS_ONE_POINT_FIVE, STOPBITS_TWO
    )
    print(f"порт {ser.name} открыт ")
except Exception as error:
    print(f"Ошибка открытия порта. Причина: {error}")
    exit()


try:
    time.sleep(1)

    test_message = "Hnoisdfoigsaighoi"
    print(f"Отправляем: '{test_message}'")

    encode_message = test_message.encode("utf-8")
    ser.write(encode_message)

    time.sleep(0.1)

    print("проверкак данных")

    bytes_avalible = ser.in_waiting  # сотрим пришло ли что то сейчас и сколько 

    if bytes_avalible > 0:
        print(f"В буфере обмена доступно байт: {bytes_avalible}")

        data = ser.read(bytes_avalible)

        decode_data = data.decode("utf-8")
        print(f"успешно полученно {decode_data}")

        if test_message == decode_data:
            print("тесто пройлен все хорошо ")
        else:
            print("Данные пришли, но они искажены")
    else:
        print("буфер пуст")
    
finally:
    if "ser" in locals() and ser.is_open:
        ser.close()
        print("\nПорт закрыт. ")
    
        