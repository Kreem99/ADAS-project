import numpy as np
import RPi.GPIO as GPIO
import glob as gb
import cv2
import keras
import os
import time
import threading


code = {'Fog':0 ,'Rain':1 ,'Normal':2}



def getcode(n):
    for x, y in code.items():
        if n == y:
            return x

def load_model(model_path):
    return keras.models.load_model(model_path)
    
    
#digital image processing => DIP
def DIP(image_path, s=200):
    X_pred = []
    image = cv2.imread(image_path)
    image_array = cv2.resize(image, (s, s))
    X_pred.append(list(image_array))
    X_pred_array = np.array(X_pred)
    return X_pred_array



def predict(model, X_pred_array):
    y_result = model.predict(X_pred_array)
    cod = getcode(np.argmax(y_result[0]))
    return cod
    
    

def setup_GPIO(TR_R=7, ECO_R=11, TR_L=3, ECO_L=5, LED=13, DC_R=15, DC_L=19, Speed=12):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(TR_R, GPIO.OUT)
    GPIO.setup(ECO_R, GPIO.IN)
    GPIO.setup(TR_L, GPIO.OUT)
    GPIO.setup(ECO_L, GPIO.IN)
    GPIO.setup(LED, GPIO.OUT)
    GPIO.setup(DC_R, GPIO.OUT)
    GPIO.setup(DC_L, GPIO.OUT)
    GPIO.setup(Speed, GPIO.OUT)
    Duty_cy = GPIO.PWM(Speed, 30)
    Duty_cy.start(90)

sound_lock = threading.Lock() #To prevent program from duplicate threading
def play_sound(case_mode_code):
    sound_lock.acquire()
    
    if (case_mode_code==23):
        os.system("aplay /home/raspi/Documents/sounds/P_L.wav")
    
    elif (case_mode_code==22):
        os.system("aplay /home/raspi/Documents/sounds/B_L.wav")
        os.system("aplay /home/raspi/Documents/sounds/W_A.wav")
    elif (case_mode_code==21):
        os.system("aplay /home/raspi/Documents/sounds/W_A.wav")
    elif (case_mode_code==33):
        os.system("aplay /home/raspi/Documents/sounds/P_R.wav")
    elif (case_mode_code==32):
        os.system("aplay /home/raspi/Documents/sounds/B_R.wav")
        os.system("aplay /home/raspi/Documents/sounds/W_A.wav")
    elif (case_mode_code==31):
        os.system("aplay /home/raspi/Documents/sounds/W_A.wav")
    elif (case_mode_code==43):
        os.system("aplay /home/raspi/Documents/sounds/P_F.wav")
    elif (case_mode_code==42):
        os.system("aplay /home/raspi/Documents/sounds/B_F.wav")
        os.system("aplay /home/raspi/Documents/sounds/W_A.wav")
    else:
        os.system("aplay /home/raspi/Documents/sounds/W_A.wav")
    sound_lock.release()

def get_pulse_duration(TR_pin, ECO_pin):
    GPIO.output(TR_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TR_pin, GPIO.LOW)

    while GPIO.input(ECO_pin) == 0:
        pulse_start = time.time()

    while GPIO.input(ECO_pin) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = round((pulse_duration * 17150), 2)

    return distance


def detect_obstacle():
    threshold_value=100 #readable distance
    mode3_distance=95
    mode2_distance=50
    DS_R = get_pulse_duration(TR_R, ECO_R)
    DS_L = get_pulse_duration(TR_L, ECO_L)
    
    if (DS_R > threshold_value and DS_L > threshold_value):#case1
        time.sleep(1)
        return 0
    
    elif (DS_R <= threshold_value and DS_L <= threshold_value):
        F = min(DS_R, DS_L)
        
        if (F >= mode3_distance):#mode3
            print("There is an object in front after %s cm" % F)
            return 23
            
        elif (F >= mode2_distance):#mode2
            print("There is an object in front after %s cm" % F)
            return 22
            
        else:#mode1
            print("There is an object in front after %s cm" % F)
            return 21
            
    elif (DS_R > threshold_value and DS_L <= threshold_value):#case3
    
        if (DS_L >= mode3_distance):#mode3
            print("There is an object on the left after %s cm" % DS_L)
            return 33
            
        elif (DS_L >= mode2_distance):#mode2
            print("There is an object on the left after %s cm" % DS_L)
            return 32
            
        else:#mode1
            print("There is an object on the left after %s cm" % DS_L)
            return 31
                    
    else:
    
        if (DS_R >= mode3_distance):#mode3
            print("There is an object on the right after %s cm" % DS_R)
            return 43
            
        elif (DS_R >= mode2_distance):#mode2
            print("There is an object on the right after %s cm" % DS_R)
            return 42
            
        else:#mode1
            print("There is an object on the right after %s cm" % DS_R)
            return 41


while True:
    model_path='/home/raspi/Desktop/imatest/modsave.h5'
    image_path='/home/raspi/Desktop/images/test3.jpg'
    # Capture image
    os.system("fswebcam -r 640x480 -S 20 --set brightness=100% " + image_path)

    # Predict image
    model = load_model(model_path)
    X_pred_array = DIP(image_path)#digital image processing => DIP
    cod = predict(model, X_pred_array)
    print(f"this is {cod}")

    if (cod == "Normal"):
        time.sleep(10)
    else:
        start_time = time.time()
        while (time.time() - start_time) < 20: #Make sure this will work 20 second and then will check weather again by take new picture for predict
            # GPIO setup & make LED,DC left and Right Running at idle mode (case 1)
            setup_GPIO()
            GPIO.output(LED, 1)
            GPIO.output(DC_L, 1)
            GPIO.output(DC_R, 1)
        
            # Obstacle detection code
            x=detect_obstacle()
            if x==0:
                pass
            else:
                
                threading.Thread(target=play_sound, args=(x,)).start()
                
                #These make Hazard light flashing in mode 1 after immediate breaking 
                if x in [41, 31, 21]:
                    for i in range(11):
                        GPIO.output(LED,0)
                        time.sleep(0.5)
                        GPIO.output(LED,1)
                        time.sleep(0.5)
            time.sleep(0.5) #To control speed of measure distance
    # Clean up GPIO
    GPIO.cleanup()