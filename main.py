import serial
import cv2
import mediapipe as mp
import time as t

# Inicializacion del puerto serial 
#ser = serial.Serial('/dev/cu.usbserial-0001', 115200)

# Inicializacion de variables
angulos_servo = [90, 90, 90, 90, 60]  # base, brazo1, brazo2, brazo3, agarre
estado_garra = False  # Estado de la garra: False = abierta (60), True = cerrada (0)
reset_gesto_garra = False

# Inicializacion de los módulos de MediaPipe
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_manos = mp.solutions.hands

# Inicializacion de la camara
camara = cv2.VideoCapture(0)

# Funciones de utilidad
def apunta_abajo(dedo_indice_y, muneca_y, dedo_medio_y, dedo_anular_y):
    return dedo_indice_y > muneca_y and dedo_indice_y > dedo_medio_y and dedo_indice_y > dedo_anular_y

def apunta_arriba(dedo_indice_y, muneca_y, dedo_medio_y, dedo_anular_y):
    return dedo_indice_y < muneca_y and dedo_indice_y < dedo_medio_y and dedo_indice_y < dedo_anular_y

def limitar_valor(valor, minimo, maximo):
    return max(min(maximo, valor), minimo)

def estan_cerca(valor1, valor2, tolerancia):
    return abs(valor1 - valor2) < tolerancia

def giro_base(dedo_anular_x, muneca_x, tolerancia):
    if abs(dedo_anular_x - muneca_x) > tolerancia:
        if dedo_anular_x > muneca_x:
            return "izquierda"
        else:
            return "derecha"
    else:
        return "centro"

# Bucle principal
with mp_manos.Hands(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5) as manos:
    # Tiempos de retraso para que no se actualicen los ángulos de los servos muy rápido
    tiempo_brazo1_abajo = t.time()
    tiempo_brazo1_arriba = t.time()
    tiempo_base = t.time()
    tiempo_garra = t.time()  # Tiempo para control de la garra
    retraso = 0.1  # Retraso de 100 milisegundos, mas corto si se necesita una respuesta más rápida

    while camara.isOpened():
        hay_mano, imagen = camara.read()
        if not hay_mano:
            print("No se detecta una mano.")
            continue

        imagen_rgb = cv2.cvtColor(imagen, cv2.COLOR_BGR2RGB)
        resultado = manos.process(imagen_rgb)

        if resultado.multi_hand_landmarks:
            for landmarks in resultado.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    imagen, 
                    landmarks, 
                    mp_manos.HAND_CONNECTIONS, 
                    mp_drawing_styles.get_default_hand_landmarks_style(), 
                    mp_drawing_styles.get_default_hand_connections_style()
                )

                muneca_x = 1000 * landmarks.landmark[0].x
                dedo_gordo_x = 1000 * landmarks.landmark[4].x
                dedo_menique_x = 1000 * landmarks.landmark[20].x
                dedo_indice_x = 1000 * landmarks.landmark[8].x
                dedo_medio_x = 1000 * landmarks.landmark[12].x
                dedo_anular_x = 1000 * landmarks.landmark[16].x

                muneca_y = 1000 * landmarks.landmark[0].y
                dedo_indice_y = 1000 * landmarks.landmark[8].y
                dedo_menique_y = 1000 * landmarks.landmark[20].y
                dedo_medio_y = 1000 * landmarks.landmark[12].y
                dedo_anular_y = 1000 * landmarks.landmark[16].y
                dedo_gordo_y = 1000 * landmarks.landmark[4].y

                # Control de la garra tipo flip-flop set reset
                if estan_cerca(dedo_gordo_x, dedo_menique_x, 50) and not reset_gesto_garra and t.time() - tiempo_garra > retraso:
                    reset_gesto_garra = True
                    estado_garra = not estado_garra
                    tiempo_garra = t.time()
                    if estado_garra:
                        angulos_servo[4] = 0
                    else:
                        angulos_servo[4] = 60
                else:
                    reset_gesto_garra = False
                    
                    

                # Actualizamos el ángulo del brazo 1 si es necesario
                if apunta_abajo(dedo_indice_y, muneca_y, dedo_medio_y, dedo_anular_y):
                    if t.time() - tiempo_brazo1_abajo > retraso:
                        
                        angulos_servo[1] = limitar_valor(angulos_servo[1] - 1, 0, 180)
                        tiempo_brazo1_abajo = t.time()
                elif apunta_arriba(dedo_indice_y, muneca_y, dedo_medio_y, dedo_anular_y):
                    if t.time() - tiempo_brazo1_arriba > retraso:
                        
                        angulos_servo[1] = limitar_valor(angulos_servo[1] + 1, 0, 180)
                        tiempo_brazo1_arriba = t.time()

                # Actualizamos la base si es necesario
                movimiento_base = giro_base(dedo_anular_x, muneca_x, 100)
                if movimiento_base == "izquierda" and t.time() - tiempo_base > retraso:
                    
                    angulos_servo[0] = limitar_valor(angulos_servo[0] - 1, 0, 180)
                    tiempo_base = t.time()
                elif movimiento_base == "derecha" and t.time() - tiempo_base > retraso:
                    
                    angulos_servo[0] = limitar_valor(angulos_servo[0] + 1, 0, 180)
                    tiempo_base = t.time()

                #ser.write(bytearray(angulos_servo))
                print(angulos_servo)
       
        cv2.imshow('MediaPipe Hands', imagen)
        if cv2.waitKey(5) & 0xFF == 27:
            break

camara.release()
cv2.destroyAllWindows()
