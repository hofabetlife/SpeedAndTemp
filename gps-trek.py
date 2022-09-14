import time
import serial
import pynmea2
import RPi.GPIO as gpio
import sqlite3
import os
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from time import sleep
import statistics

#термопара подключена к gpio4
#экран подключен по i2c gpio2-3
#gps по uart gpio14-15

gpio.setmode(gpio.BCM)
port = "/dev/ttyAMA0" # последовательный порт, к которому подключена плата gps
#нужно ослеживать последнюю координату и ее время
time_before = 0.0
pos_before_x = 0.0
pos_before_y = 0.0
tim_now = ''
day_now = ''
t_tue = False
# ктермопаре
os.system('modprobe w1-gpio') 
os.system('modprobe w1-therm')
gpio.setup(21,gpio.OUT)
#по температуре
temp_c1 = 0.0
temp_c2 = 0.0
RST = 0
disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST)
disp.begin()
disp.clear()
disp.display()
width = 128
height = 64
image1 = Image.new('1', (width, height))
draw = ImageDraw.Draw(image1)
draw.rectangle((0,0,width,height), outline=0, fill=0)
padding = -2
top = padding
bottom = height-padding
x = 0
font = ImageFont.truetype('/home/pupa/ra.ttf', 26)
ser = serial.Serial(port, baudrate = 9600, timeout = 2) # создаем объект для работы с последовательным портом
geo_list = list()

def get_temp(): # функция для считывания значения температуры с датчика
    try:
        file = open('/sys/bus/w1/devices/28-3c0ff648c9e3/w1_slave', 'r') # открываем файл
        lines = file.readlines() # считываем строки файла 
        file.close() # закрываем файл 
        trimmed_data = lines[1].find('t=') # находим символы "t=" в строке
        if trimmed_data != -1:
            temp_string = lines[1][trimmed_data+2:] # вырезаем из строки необходимое нам значение температуры
            temp_c1 = float(temp_string) / 1000.0 # делим считанное значение температуры на 1000 чтобы получить правильное значение температуры
    except:
        temp_c1 = "null"
    try:
        file = open('/sys/bus/w1/devices/28-3c3bf648f675/w1_slave', 'r') # открываем файл
        lines = file.readlines() # считываем строки файла 
        file.close() # закрываем файл 
        trimmed_data = lines[1].find('t=') # находим символы "t=" в строке
        if trimmed_data != -1:
            temp_string = lines[1][trimmed_data+2:] # вырезаем из строки необходимое нам значение температуры
            temp_c2 = float(temp_string) / 1000.0 # делим считанное значение температуры на 1000 чтобы получить правильное значение температуры
    except:
        temp_c2 = "null"
    return [str(temp_c1), str(temp_c2)]  
def pos_and_time(pos_lat, pos_long):
    time_before = time.time()
    pos_before_x = pos_lat
    pos_before_y = pos_long
def sveto(): #чтобы мегало
    gpio.setup(21,gpio.OUT)         # конфигурируем контакт pin40 в качестве цифрового выхода
    gpio.output(21,1)             # включаем светодиод (подаем на pin40 напряжение с уровнем HIGH)
    time.sleep(0.2)               # задержка на 1 секунду
def db_write(pos_1, pos_2, day_now, tim_now): #выводим текст в фаил
    
    connection = sqlite3.connect('gps_data.db')
    cursor = connection.cursor()
    cursor.execute('''CREATE TABLE IF NOT EXISTS Shows
    (lat INTEGER, long INTEGER, time timestamp)''')
    # вставить данные
    sqlite_insert_with_param = """INSERT INTO 'Shows'
                    ('lat', 'long', 'time')
                 VALUES (?, ?, ?);"""
    data_tuple = (pos_1, pos_2, day_now+tim_now)
    cursor.execute(sqlite_insert_with_param, data_tuple)
    connection.commit()
    connection.close()  
def longi_lati(msg): #считываем широту и долготу 
    latval = msg.lat
    pos_lat = float(latval[0])*10+float(latval[1])
    pos_lat +=(float(latval[2])*10+float(latval[3])+float(latval[5])/10+float(latval[6])/100+float(latval[7])/1000+float(latval[8])/10000+float(latval[9])/100000)/60
    longval = msg.lon
    pos_long = float(longval[0])*100+float(longval[1])*10+float(longval[2])
    pos_long += (float(longval[3])*10+float(longval[4])+float(longval[6])/10+float(longval[7])/100+float(longval[8])/1000+float(longval[9])/10000+float(longval[10])/100000)/60
    return pos_lat, pos_long
def data_ok(data): #проверка есть ли в строке нужное значение
    data = str(data)
    if len(data) > 10: 
        if data.find("$GPGGA") > 0 or data.find("$GPRMC") > 0:
            return True
        else:
            return False
    else:
        return False
def display(speed): #рисуем на дисплее
    draw.rectangle((0,0,width,height), outline=0, fill=0)
    list =  get_temp()
    # Draw Some Text
    tmp = "T: "+str(list[0])+"°C"
    draw.text((x, top),       tmp ,  font=font, fill=255)
    tmp = "T: "+str(list[1])+"°C"
    draw.text((x, top+20),     tmp, font=font, fill=255)
    draw.text((x, top+40),     speed, font=font, fill=255)
    disp.clear()
    # Display image.
    disp.image(image1)
    disp.display()
def filtr(speed): #усредняем значение скорости
    if len(geo_list) >= 10:
        speeds = statistics.median(geo_list)
        geo_list.clear()
        return speeds
    else:
        geo_list.append(speed)
        return 0
    
def main(): # основной цикл
    display("Load")
    speed_max = 0.0
    while 1:
        try:
            data = ser.readline()
            #проверка есть ли в строке нужное значение
            if data_ok(data) == False: continue
            data_debila = data
            data_debila = data_debila.decode('utf-8')
            data_debila = data_debila.split(",")
            if '$GPGGA' in data_debila[0] and data_debila[2] != '': #мы будем извлекать широту и долготу из строки GPGGA в данных NMEA
                try:
                    msg = pynmea2.parse(data.decode('utf-8'))
                    pos_lat, pos_long = longi_lati(msg)
                    if t_tue:
                        db_write(pos_lat, pos_long, day_now, tim_now)#сохроняем координаты в бд
                    pos_and_time(pos_lat, pos_long)#зафиксируем текущее время и координаты
                    sveto() #вкл светодиод
                except:
                    display("ERROR1")
                    print("Что то с данными о месте")
                    gpio.output(21,0)
            if '$GPRMC' in data_debila[0] and data_debila[1] != '':
                try:
                    Time_spl = str(data_debila[1])
                    tim_now = Time_spl[0]+Time_spl[1]+":"+Time_spl[2]+Time_spl[3]
                    Time_spl = str(data_debila[9])
                    day_now = Time_spl[0]+Time_spl[1]+"."+Time_spl[2]+Time_spl[3]+"."+Time_spl[4]+Time_spl[5]+","
                    speeds = float(data_debila[7])*1.852    
                    #if speed_max < speeds:
                    #    speed_max = speeds #в текущей версии не юзаеться
                    speeds = filtr(speeds)
                    if speeds > 0:
                        spd = str(round(speeds, 2)) + 'km\h'
                        display(spd)
                    t_tue = True
                except:
                    display("ERROR2")
                    print("Что то с данными о времени")
                    t_tue = False
        except:
            port = "/dev/ttyAMA0" # последовательный порт, к которому подключена плата Raspberry Pi
            ser = serial.Serial(port, baudrate = 9600, timeout = 2)
            gpio.output(21,0)

if __name__ == '__main__':
    main() 
    
