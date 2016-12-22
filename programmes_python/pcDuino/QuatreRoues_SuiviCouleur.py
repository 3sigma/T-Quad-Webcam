#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme de contrôle du robot T-Quad avec 4 roues holonomes (roues Mecanum)
# disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Suivi de couleur
#
# Auteur: 3Sigma
# Version 1.1.0 - 22/12/2016
##################################################################################

# Importe les fonctions Arduino pour Python
from pyduino_pcduino import *

# Imports pour la communication i2c avec l'Arduino Mega
from mega import Mega
mega = Mega()

import time, sched
import os
import threading
import signal
import json
import sys

# Pour la détection d'adresse IP
import socket
import fcntl
import struct

# Pour le serveur de socket
import tornado.httpserver
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web
import tornado.websocket
import tornado.template

# Gestion de l'IMU
import FaBo9Axis_MPU9250

# Imports pour OpenCV
import cv2
import numpy as np
import colorsys

Nmoy = 1

omegaArriereDroit = 0.
codeurArriereDroitDeltaPos = 0
codeurArriereDroitDeltaPosPrec = 0

omegaArriereGauche = 0.
codeurArriereGaucheDeltaPos = 0
codeurArriereGaucheDeltaPosPrec = 0

omegaAvantDroit = 0.
codeurAvantDroitDeltaPos = 0
codeurAvantDroitDeltaPosPrec = 0

omegaAvantGauche = 0.
codeurAvantGaucheDeltaPos = 0
codeurAvantGaucheDeltaPosPrec = 0

# Tension effectivement appliquée
commandeArriereDroit = 0.
commandeArriereGauche = 0.
commandeAvantDroit = 0.
commandeAvantGauche = 0.

# Saturations
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur

# Paramètres mécaniques
R = 0.0225 # Rayon d'une roue
W = 0.18 # Ecart entre le centre de rotation du robot et les roues

# Variables utilisées pour les données reçues
# Couleurs initiale et précédente
couleur = "FF9600"
couleurPrec = "FF9600"
# Rayon de l'objet (balle de tennis)
Robjet = 0.035

# Timeout de réception des données
timeout = 2
timeLastReceived = 0
timedOut = False

T0 = time.time()
dt = 0.01
tprec = time.time()
tdebut = 0
# Création d'un scheduler pour exécuter des opérations à cadence fixe
s = sched.scheduler(time.time, time.sleep)

# Commande des moteurs
commandeRot = 0.
tcommande = T0

# Mesure de la tension de la batterie
# On la contraint à être supérieure à 7V, pour éviter une division par
# zéro en cas de problème quelconque
idecimLectureTension = 0
decimLectureTension = 6000
decimErreurLectureTension = 100

lectureTensionOK = False
tensionAlim = 7.4
while not lectureTensionOK:
    try:
        tensionAlim = max(7.0, float(mega.read_battery_millivolts()) / 1000.)
        lectureTensionOK = True
    except:
        print("Erreur lecture tension")

# Déclaration pour l'IMU
gz = 0.
intgz = 0.
offset_gyro = 0.

# Initialisation OpenCV
# Résolution souhaitée pour l'image. Remarque: la résolution native de l'image est 640x480, mais
# cette résolution conduit à un temps de latence assez important. Il est préférable d'utiliser
# une résolution de 320x240
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
# Rayon minimum du cercle entourant l'objet dont on a reconnu la couleur
MIN_RADIUS = 2
# Initialisation de la Webcam
cam = cv2.VideoCapture(0)
cam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
camWidth = cam.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)
camHeight = cam.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)
print "Camera initialized: (" + str(camWidth) + ", " + str(camHeight) + ")"
# Compteur de boucle
i = 0
# Taille de l'objet
radius = 0.
# Position de l'objet
center = (0,0)
# Position précédente
positionPrec = CAMERA_WIDTH / 2
# Ecart angulaire
ecart_angulaire = 0.

#--- setup --- 
def setup():
    global imu, offset_gyro
    
    # Initialisation de l'IMU
    initIMU_OK = False
    while not initIMU_OK:
        try:
            imu = FaBo9Axis_MPU9250.MPU9250()
            initIMU_OK = True
        except:
            print("Erreur init IMU")
    
    # Initialisation des moteurs
    CommandeMoteurs(0, 0, 0, 0)
    
    # Calibration du gyro sur 100 mesures
    i = 0
    somme_gyro = 0.
    while (i < 100):
        try:
            gyro = imu.readGyro()
            gz = gyro['z'] * math.pi / 180
            somme_gyro = somme_gyro + gz
            i = i + 1
        except:
            #print("Erreur lecture IMU")
            pass
    offset_gyro = somme_gyro/100.
    print "Offset gyro", offset_gyro
    i = 0
        
 
# -- fin setup -- 
 
# -- loop -- 
def loop():
    CalculVitesse()

# -- fin loop --

def fois255entier(x):
    return int(round(x * 255))

def SuiviCouleur():
    global MIN_RADIUS, CAMERA_WIDTH, cam, i, center, radius, distance, ecart_angulaire, couleur, Robjet

    # Conversion de la couleur de l'hexadécimal en HSV (Teinte - Saturation - Luminosité)
    couleurRGB = struct.unpack('BBB',couleur.decode('hex'))
    couleurHSV = map(fois255entier, colorsys.rgb_to_hsv(couleurRGB[0]/255.,couleurRGB[1]/255.,couleurRGB[2]/255.))

    # Définition des seuils de reconnaissance (seule la teinte est réglable de -/+ 10 autour de la valeur nominale
    THRESHOLD_LOW = (max(0,couleurHSV[0] - 10), 20, 20);
    THRESHOLD_HIGH = (min(255,couleurHSV[0] + 10), 255, 255);

    # Lecture d'une image
    ret_val, img = cam.read()
    
    # Suppression de bruit
    img_filter = cv2.GaussianBlur(img.copy(), (3, 3), 0)

    # Conversion de l'image en HSV
    img_filter = cv2.cvtColor(img_filter, cv2.COLOR_BGR2HSV)

    # Conversion de l'image en binaire (les pixels dans l'intervalle de couleur sélectionné
    # sont convertis en blanc, les autres en noir)
    img_binary = cv2.inRange(img_filter.copy(), np.array(THRESHOLD_LOW), np.array(THRESHOLD_HIGH))

    # Dilatation de l'image pour augmenter la taille des blobs
    img_binary = cv2.dilate(img_binary, None, iterations = 1)

    # Détection du centre de l'objet à partir des contours. Voir:
    # http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
    img_contours = img_binary.copy()
    contours = cv2.findContours(img_contours, cv2.RETR_EXTERNAL, \
        cv2.CHAIN_APPROX_SIMPLE)[-2]

    # Recherche du contour le plus grand et calcul du cercle l'entourant
    center = None
    radius = 0
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        if M["m00"] > 0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if radius < MIN_RADIUS:
                center = None

    # Calculs sur le plus grand contour détecté
    if center != None:
        #print str(center) + " " + str(radius)
        # Distance à l'objet
        distance = Robjet / math.tan(0.1186823645e1 * radius / CAMERA_WIDTH)
        # L'angle de vue de la Webcam est 68 degrés
        # On considère que l'écart angulaire est nul si le rayon de l'objet est inférieur à 10
        # (dans ce cas, l'objet n'est probablement pas dans le champ de vision)
        if (radius > 10):
            ecart_angulaire = (34. * (center[0] - (CAMERA_WIDTH / 2)) / (CAMERA_WIDTH / 2)) * (math.pi / 180.)
        else:
            ecart_angulaire = 0.

    # Ajout à l'image d'un cercle vert autour du plus grandd contour
    if center != None:
        cv2.circle(img, center, int(round(radius)), (0, 255, 0))

    # Ecriture de l'image dans un répertoire spécifique.
    # Le logiciel mjpg_streamer, qui est exécuté en parallèle lors du démarrage de ce programme Python
    # par l'application MyViz (ou par son script .sh associé), stream une vidéo à partir des images lues
    # dans ce répertoire
    cv2.imwrite('/root/programmes_python/jpg/{0:05d}.jpg'.format(i),img)
    i += 1
    cv2.waitKey(1)
        

def CalculVitesse():
    global omegaArriereDroit, omegaArriereGauche, omegaAvantDroit, omegaAvantGauche, timeLastReceived, timeout, timedOut, \
        tdebut, codeurArriereDroitDeltaPos, codeurArriereGaucheDeltaPos, codeurAvantDroitDeltaPos, codeurAvantGaucheDeltaPos, \
        commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche, \
        codeurArriereDroitDeltaPosPrec, codeurArriereGaucheDeltaPosPrec, codeurAvantDroitDeltaPosPrec, codeurAvantGaucheDeltaPosPrec, tprec, \
        idecimLectureTension, decimLectureTension, decimErreurLectureTension, tensionAlim, \
        imu, gz, intgz, R, W, commandeRot, offset_gyro, ecart_angulaire, tcommande
    
    tdebut = time.time()
    
    # Exécution de la fonction de suivi de couleur. Le résultat du calul est l'écart angulaire entre l'axe de la Webcam et
    # l'objet coloré
    SuiviCouleur()
    
    # Mesure de la vitesse des moteurs grâce aux codeurs incrémentaux
    try:
        codeursDeltaPos = mega.read_codeursDeltaPos()
        codeurArriereDroitDeltaPos = codeursDeltaPos[0]
        codeurArriereGaucheDeltaPos = codeursDeltaPos[1]
        codeurAvantDroitDeltaPos = codeursDeltaPos[2]
        codeurAvantGaucheDeltaPos = codeursDeltaPos[3]
        
        # Suppression de mesures aberrantes
        if (abs(codeurArriereDroitDeltaPos - codeurArriereDroitDeltaPosPrec) > 10) or (abs(codeurArriereGaucheDeltaPos - codeurArriereGaucheDeltaPosPrec) > 10) or (abs(codeurAvantDroitDeltaPos - codeurAvantDroitDeltaPosPrec) > 10) or (abs(codeurAvantGaucheDeltaPos - codeurAvantGaucheDeltaPosPrec) > 10):
            codeurArriereDroitDeltaPos = codeurArriereDroitDeltaPosPrec
            codeurArriereGaucheDeltaPos = codeurArriereGaucheDeltaPosPrec
            codeurAvantDroitDeltaPos = codeurAvantDroitDeltaPosPrec
            codeurAvantGaucheDeltaPos = codeurAvantGaucheDeltaPosPrec


        codeurArriereDroitDeltaPosPrec = codeurArriereDroitDeltaPos
        codeurArriereGaucheDeltaPosPrec = codeurArriereGaucheDeltaPos
        codeurAvantDroitDeltaPosPrec = codeurAvantDroitDeltaPos
        codeurAvantGaucheDeltaPosPrec = codeurAvantGaucheDeltaPos
    except:
        #print "Error getting data"
        codeurArriereDroitDeltaPos = codeurArriereDroitDeltaPosPrec
        codeurArriereGaucheDeltaPos = codeurArriereGaucheDeltaPosPrec
        codeurAvantDroitDeltaPos = codeurAvantDroitDeltaPosPrec
        codeurAvantGaucheDeltaPos = codeurAvantGaucheDeltaPosPrec
    
    # On utilise ici dt car c'est à cette cadence (10 ms) que les mesures des delta codeurs
    # sont effectuées sur l'Arduino
    omegaArriereDroit = -2 * ((2 * 3.141592 * codeurArriereDroitDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    omegaArriereGauche = 2 * ((2 * 3.141592 * codeurArriereGaucheDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    omegaAvantDroit = -2 * ((2 * 3.141592 * codeurAvantDroitDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    omegaAvantGauche = 2 * ((2 * 3.141592 * codeurAvantGaucheDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    
    # Mesure de la vitesse de rotation par odométrie (non utilisé pour l'instant)
    ximes = (omegaArriereDroit - omegaArriereGauche + omegaAvantDroit - omegaAvantGauche) * R / W / 2
    
    dt2 = time.time() - tprec
    tprec = time.time()
            
    # Lecture de la vitesse de rotation autour de la verticale (non utilisé pour l'instant)
    try:
        gyro = imu.readGyro()
        gz = gyro['z'] * math.pi / 180 - offset_gyro
        if (ximes == 0.):
            offset_gyro = offset_gyro + gz
    except:
        #print("Erreur lecture IMU")
        pass
    
    # On utilise par la suite dt2 car c'est l'écart de temps entre deux calculs
    # Intégration de la vitesse de rotation pour avoir l'angle (non utilisé pour l'instant)
    intgz = intgz + gz * dt2
                
    # Si on n'a pas reçu de données depuis un certain temps, celles-ci sont annulées
    if (time.time()-timeLastReceived) > timeout and not timedOut:
        timedOut = True
        
    if timedOut:
        commandeRot = 0.
    else:
        # On tourne de 6 degrés par volt
        # A faire: ajuster ce rapport (6 degrés / V) en fonction de la mesure effectuée par le gyro
        if (time.time() - tcommande) > 0.2:
            commandeRot = -(180. * ecart_angulaire / math.pi) / 6.
            tcommande = time.time()
        else:
            commandeRot = 0.
        
    # Transformation des commandes longitudinales et de rotation en tension moteurs
    commandeArriereDroit = -commandeRot # Tension négative pour faire tourner positivement ce moteur
    commandeArriereGauche = -commandeRot
    commandeAvantDroit = -commandeRot # Tension négative pour faire tourner positivement ce moteur
    commandeAvantGauche = -commandeRot
        
    CommandeMoteurs(commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche)
    
    # Lecture de la tension d'alimentation (non utilisé pour l'instant)
    if idecimLectureTension >= decimLectureTension:
        try:
            tensionAlim = max(7.0, float(mega.read_battery_millivolts()) / 1000.)
            idecimLectureTension = 0
        except:
            # On recommence la lecture dans decimErreurLectureTension * dt
            idecimLectureTension = idecimLectureTension - decimErreurLectureTension
            #print("Erreur lecture tension dans Loop")
    else:
        idecimLectureTension = idecimLectureTension + 1    
        
    
    # Suggestions d'amélioration:
    # - utiliser l'angle de rotation mesuré par le gyro pour recaler le rappport de 6 degrés / V
    # - faire un suivi de distance
    
    #print time.time() - tdebut

    
def CommandeMoteurs(commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    global tensionAlim
    
    # L'ensemble pont en H + moteur pourrait ne pas être linéaire
    tensionArriereDroit = commandeArriereDroit
    tensionArriereGauche = commandeArriereGauche
    tensionAvantDroit = commandeAvantDroit
    tensionAvantGauche = commandeAvantGauche

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int_ArriereDroit = int(255 * tensionArriereDroit / tensionAlim)
    tension_int_ArriereGauche = int(255 * tensionArriereGauche / tensionAlim)
    tension_int_AvantDroit = int(255 * tensionAvantDroit / tensionAlim)
    tension_int_AvantGauche = int(255 * tensionAvantGauche / tensionAlim)

    # Saturation par sécurité
    if (tension_int_ArriereDroit > 255):
        tension_int_ArriereDroit = 255

    if (tension_int_ArriereDroit < -255):
        tension_int_ArriereDroit = -255

    if (tension_int_ArriereGauche > 255):
        tension_int_ArriereGauche = 255

    if (tension_int_ArriereGauche < -255):
        tension_int_ArriereGauche = -255

    if (tension_int_AvantDroit > 255):
        tension_int_AvantDroit = 255

    if (tension_int_AvantDroit < -255):
        tension_int_AvantDroit = -255

    if (tension_int_AvantGauche > 255):
        tension_int_AvantGauche = 255

    if (tension_int_AvantGauche < -255):
        tension_int_AvantGauche = -255

    # Commande PWM
    try:
        mega.moteursArriere(tension_int_ArriereDroit, tension_int_ArriereGauche)
        mega.moteursAvant(tension_int_AvantDroit, tension_int_AvantGauche)
        mega.moteursCRC(tension_int_ArriereDroit + tension_int_ArriereGauche, tension_int_AvantDroit + tension_int_AvantGauche)
    except:
        pass
        #print "Erreur moteurs"

    
def emitData():
    global tprec
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    #delay(5000)
    tprec = time.time()
    while not noLoop: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 100)
        self.callback.start()
    

    def on_message(self, message):
        global couleur, timeLastReceived, timedOut, couleurPrec, Robjet
            
        jsonMessage = json.loads(message)
        
        # Annulation du timeout de réception des données
        timeLastReceived = time.time()
        timedOut = False;
        
        if jsonMessage.get('couleur') != None:
            couleur = jsonMessage.get('couleur')[1:]
            couleurPrec = couleur
        if jsonMessage.get('Robjet') != None:
            Robjet = 0.01 * float(jsonMessage.get('Robjet'))
                        
        if not socketOK:
            couleur = couleurPrec


    def on_close(self):
        global socketOK
        print 'connection closed...'
        socketOK = False

    def sendToSocket(self):
        global socketOK, omegaArriereDroit, omegaArriereGauche, omegaAvantDroit, omegaAvantGauche, \
            gz, intgz, commandeRot, center, radius, distance, positionPrec, ecart_angulaire
            
        if (center == None):
            position = positionPrec
        else:
            position = center[0]
            positionPrec = position
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({'Temps':("%.2f" % tcourant), \
                                'ecart_angulaire':("%.2f" % (ecart_angulaire*180/math.pi)), \
                                'intgz':("%.2f" % (intgz*180/math.pi)), \
                                'commande_rotation':("%.2f" % commandeRot), \
                                'position':("%d" % position), \
                                'rayon':("%.2f" % radius), \
                                'distance':("%.2f" % (100 * distance)), \
                                'Raw':("%.2f" % tcourant) \
                                + "," + ("%.2f" % (ecart_angulaire*180/math.pi)) \
                                + "," + ("%.2f" % (intgz*180/math.pi)) \
                                + "," + ("%.2f" % commandeRot) \
                                + "," + ("%d" % position) \
                                + "," + ("%.2f" % radius) \
                                + "," + ("%.2f" % (100 * distance)) \
                                })
                                
        if socketOK:
            try:
                self.write_message(aEnvoyer)
            except:
                pass
            
    def check_origin(self, origin):
        # Voir http://www.tornadoweb.org/en/stable/websocket.html#tornado.websocket.WebSocketHandler.check_origin
        # et http://www.arundhaj.com/blog/tornado-error-during-websocket-handshake.html
        return True        

    
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
    
application = tornado.web.Application([
    (r'/ws', WSHandler)
])

def startTornado():
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9090)
    tornado.ioloop.IOLoop.instance().start()


# Gestion du CTRL-C
def signal_handler(signal, frame):
    print 'Sortie du programme'
    CommandeMoteurs(0, 0, 0, 0)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

#--- obligatoire pour lancement du code -- 
if __name__=="__main__": # pour rendre le code executable 
    setup() # appelle la fonction setup
    print "Setup done."
    
    th = threading.Thread(None, emitData, None, (), {})
    th.daemon = True
    th.start()
    
    print "Starting Tornado."
    try:
        print "Connect to ws://" + get_ip_address('eth0') + ":9090/ws with Ethernet."
    except:
        pass
        
    try:
        print "Connect to ws://" + get_ip_address('wlan0') + ":9090/ws with Wifi."
    except:
        pass
    socketOK = False
    startTornado()


