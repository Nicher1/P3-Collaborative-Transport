#*********************************** GERMAN ******************************************************
#Python Beispiel-Programm zur Kommunikation/Steuerung der dyve D1 über Modbus TCP als Gateway (Ethernet basiert)
#Version 1.1

#Programmiersprache Python 3.6 
#Bitte mit dem Standard Raspian Editor "Thonny" (Raspberry PI, ab Noobs 3.2) oder idle 3.7 ausführen. 

#Steuerung einer Einzelachse
#Es muss die mitgelieferte dryve D1 Konfigurationen geladen sein: 20220428_D1-1-PC-ModbusTCP(GW)-9-V1.1.txt
#Die IP Adresse der dryve D1 muss mit der unter "s.connect" eingetragenen IP Adresse übereinstimmen. 
#Im Programm können alle Bewegungsparameter angepasst werden. Z.B. Geschwindigkeit, Beschleunigung oder die Strecke.

#Bitte immer die neueste Firmware von der Webseite www.igus.de/dryve verwenden!!!

#Für das Musterprogram wird kein Support bereitgestellt.
#Ebenfalls wird keine Verantwortung/Haftung für das Programm übernommen.
#*********************************** GERMAN ******************************************************



#*********************************** English *****************************************************
#Python sample programm demonstrating the communication/control of a dryve D1 via Modbus TCP as a Gateway (Ethernet based)
#Version 1.1

#Programing language: Python 3.6
#Please use the standard Raspian editor "Thonny" (Raspberry Pi since Noobs 3.2) or idle 3.7 to execute the program.

#Single axis control
#The supplied dryve D1 configuration must be loaded: 20220428_D1-1-PC-ModbusTCP(GW)-9-V1.1.txt
#The IP address of the dryve D1 must match with the IP address stated at "s.connect".
#All movement parameter can be adopted. E.g. speed, acceleration or position.

#Please use the latest firmware available at www.igus.eu/dryve!!!

#No support is provided for this sample program.
#No responsibility/liability will be assumed for the test program.
#*********************************** English *****************************************************



#Bibliotheken importieren
#Import libraries 
import socket
import time
import sys


#Bus-Verbindung herstellen
#Establish bus connection
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
except socket.error:
    print ('failed to create sockt')
    
s.connect(("172.31.1.101", 503))
print ('Socket created')
#Wird beim Ausfuehren des Programms nur der Speicherort und der Programmname in der Shell angezeigt, so sind die IP Adressen des Programms und der dryve D1 nicht uebereinstimmend
#When executing the program and the shell displays the storing folder and the program name, the set IP address in the program and the dryve D1 doesn't match




#Durchlauf State Machine (Handbuch: Visualisieung State Machine) 
#State Machine pass through (Manual: Visualisation State Machine)
    

# Statusword 6041h
# Status request
status = [0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2]
status_array = bytearray(status)
print(status_array)

# Controlword 6040h
# Command: Shutdown
shutdown = [0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 6, 0]
shutdown_array = bytearray(shutdown)
print(shutdown_array)

# Controlword 6040h
# Command: Switch on
switchOn = [0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 7, 0]
switchOn_array = bytearray(switchOn)
print(switchOn_array)

# Controlword 6040h
# Command: enable Operation
enableOperation = [0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 15, 0]
enableOperation_array = bytearray(enableOperation)
print(enableOperation_array)

# Controlword 6040h
# Command: reset dryve
reset = [0, 0, 0, 0, 0, 15, 0, 43,13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 0, 1]
reset_array = bytearray(reset)
print(reset_array)


#Definition der Funktion zum Senden und Empfangen von Daten
#Definition of the function to send and receive data 
def sendCommand(data):
    #Socket erzeugen und Telegram senden
    #Create socket and send request
    s.send(data)
    res = s.recv(24)
    #Ausgabe Antworttelegram 
    #Print response telegram
    print(list(res))
    return list(res)

#Shutdown Controlword senden und auf das folgende Statuswort pruefen. Pruefung auf mehrer Statuswords da mehrere Szenarien siehe Bit assignment Statusword, data package im Handbuch 
#sending Shutdown Controlword and check the following Statusword. Checking several Statuswords because of various options. look at Bit assignment Statusword, data package in user manual 
def set_shdn():
    sendCommand(shutdown_array)
    while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 6]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 22]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 2]):
        print("wait for shdn")

        #1 Sekunde Verzoegerung
        #1 second delay
        time.sleep(1)

#Switch on Disabled Controlword senden und auf das folgende Statuswort pruefen. Pruefung auf mehrer Statuswords da mehrere Szenarien siehe Bit assignment Statusword, data package im Handbuch 
#sending Switch on Disabled Controlword and check the following Statusword. Checking several Statuswords because of various options. look at Bit assignment Statusword, data package in user manual 
def set_swon():
    sendCommand(switchOn_array)
    while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 6]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 22]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 2]):
        print("wait for sw on")

        #1 Sekunde Verzoegerung
        #1 second delay
        time.sleep(1)

#Operation Enable Controlword senden und auf das folgende Statuswort pruefen. Pruefung auf mehrer Statuswords da mehrere Szenarien siehe Bit assignment Statusword, data package im Handbuch 
#Operation Enable Controlword and check the following Statusword. Checking several Statuswords because of various options. look at Bit assignment Statusword, data package in user manual 
def set_op_en():
    sendCommand(enableOperation_array)
    while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 6]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 2]):
        print("wait for op en")

        #1 Sekunde Verzoegerung
        #1 second delay
        time.sleep(1)

def init():

    #Aufruf der Funktion sendCommand zum hochfahren der State Machine mit vorher definierten Telegrammen (Handbuch: Visualisieung State Machine)
    #Call of the function sendCommand to start the State Machine with the previously defined telegrams (Manual: Visualisation State Machine)
    sendCommand(reset_array)
    set_shdn()
    set_swon()
    set_op_en()


def set_mode(mode):

    #Set operation modes in object 6060h Modes of Operation
    sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 1, mode]))
    while (sendCommand(bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1])) != [0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, mode]):

        print("wait for mode")

        #1 Sekunde Verzoegerung
        #1 second delay
        time.sleep(1)


init()

#Parametrierung der Objekte gemäß Handbuch
#Parameterization of the objects according to the manual

# 6060h Modes of Operation
#Setzen auf Homing Modus (see "def set_mode(mode):"; Byte 19 = 6)
#Set Homing mode (see "def set_mode(mode):"; Byte 19 = 6)
set_mode(6)

# 6092h_01h Feed constant Subindex 1 (Feed)
#Setzen des Vorschubs auf den Wert 6000; vgl. Handbuch  (Byte 19 = 112; Byte 20= 23)
#Set feed constant to 6000; refer to manual (Byte 19 = 112; Byte 20= 23)
sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 146, 1, 0, 0, 0, 2, 112, 23]))

# 6092h_02h Feed constant Subindex 2 (Shaft revolutions)
#Setzen der Wellenumdrehung auf 1; vgl. Handbuch (Byte 19 = 1)
#Set shaft revolutions to 1; refer to manual (Byte 19 = 1)
sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 146, 2, 0, 0, 0, 1, 1]))

# 6099h_01h Homing speeds Switch
#Vorgabe der Verfahrgeschwindigkeit beim Suchen auf den Schalter wird auf 60 U/min gesetzt (Byte 19 = 112; Byte 20 = 23))
#Speed during search for switch is set to 60 rpm (Byte 19 = 112; Byte 20 = 23))
sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 153, 1, 0, 0, 0, 2, 112, 23]))

# 6099h_02h Homing speeds Zero
#Setzen Verfahrgeschwindigkeit beim Suchen von Null auf 60 U/min (Byte 19 = 112; Byte 20 = 23))
#Set speed during Search for zero to 60 rpm (Byte 19 = 112; Byte 20 = 23))
sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 153, 2, 0, 0, 0, 2, 112, 23]))

# 609Ah Homing acceleration
#Setzen der Refernzfahrt-Beschleunigung wird auf 500 U/min² (Byte 19 = 80; Byte 20 = 195)
#Set Homing acceleration to 500 rpm/min² (Byte 19 = 80; Byte 20 = 195)
sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 154, 0, 0, 0, 0, 2, 80, 195]))

# 6040h Controlword
#Start Homing
sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 31, 0]))

#Check Statusword nach Referenziert Signal
#Check Statusword for signal referenced 
while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]):
    
        print("wait for Homing to end")
        
        #1 Sekunde Verzoegerung
        #1 second delay 
        time.sleep(1)

sendCommand(enableOperation_array)

# 6060h Modes of Operation
#Setzen auf Profile Position Mode (see "def set_mode(mode):"; Byte 19 = 1)
#Set Profile Position Mode (see "def set_mode(mode):"; Byte 19 = 1)
set_mode(1)

# 6081h Profile Velocity
#Setzen der Geschwindigkeit auf 60 U/min (Byte 19 = 112; Byte 20 = 23)
#Set velocity to 60 rpm (Byte 19 = 112; Byte 20 = 23)
sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 129, 0, 0, 0, 0, 2, 112, 23]))

# 6083h Profile Acceleration
#Setzen der Beschleunigung auf 500 U/min² (Byte 19 = 80; Byte 20 = 195)
#Set acceleration to 500 rpm/min² (Byte 19 = 80; Byte 20 = 195)
sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 131, 0, 0, 0, 0, 2, 80, 195]))

#Setzen einer Anfangsposition 
#Set start position 
setPosition0 = 0 
setPosition1 = 0  
setPosition2 = 0   
setPosition3 = 0

#Schleife Links- und Rechtslauf des Motors
#Clockwise/counter-clockwise motor movement Loop
while True:

    #Ziel-Position nach jeden Schleifendurchlauf zurücksetzen; die Variablen setPositionX werden nach jedem Schleifendurchlauf auf den Anfangswert zurückgesetzt 
    #Reset target position after each loop; the variables setPositionX are rewritten with the default value after each loop  
    sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 122, 0, 0, 0, 0, 4, setPosition0, setPosition1, setPosition2, setPosition3]))
    
    #Startbefehl zur Bewegung des Motors über Bit 4 
    #Set Bit 4 true to excecute the movoment of the motor 
    sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 31, 0]))
    
    print("go")
    
    #1 Sekunde Verzoegerung
    #1 second delay 
    time.sleep(1)

    #Check Statusword nach Ziel ereicht
    #Check Statusword for target reached 
    while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]):
        
        print("wait for next command")
        
        #1 Sekunde Verzoegerung
        #1 second delay 
        time.sleep(1)

    sendCommand(enableOperation_array)

    #Setzen der Position für den nächsten Schelifendurchlauf auf 1 Umdrehung (Byte 19 = 112; Byte 20 = 23; Byte 21 = 0; Byte 22 = 0)
    #Set position for the next loop to 1 revelution (Byte 19 = 112; Byte 20 = 23; Byte 21 = 0; Byte 22 = 0)

    setPosition0 = 112   if setPosition0 == 0 else 0
    setPosition1 = 255   if setPosition1 == 0 else 0
    setPosition2 = 0     if setPosition2 == 0 else 0
    setPosition3 = 0     if setPosition3 == 0 else 0



#Zu erwartende Antwort-Telegramme bei korrekter Programmausführung.
#Expected response telegrams with correct program execution.
    
#Socket created
#bytearray(b'\x00\x00\x00\x00\x00\r\x00+\r\x00\x00\x00`A\x00\x00\x00\x00\x02')
#bytearray(b'\x00\x00\x00\x00\x00\x0f\x00+\r\x01\x00\x00`@\x00\x00\x00\x00\x02\x06\x00')
#bytearray(b'\x00\x00\x00\x00\x00\x0f\x00+\r\x01\x00\x00`@\x00\x00\x00\x00\x02\x07\x00')
#bytearray(b'\x00\x00\x00\x00\x00\x0f\x00+\r\x01\x00\x00`@\x00\x00\x00\x00\x02\x0f\x00')
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 6]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 6]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 6]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, 0]
#wait for mode
#[0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, 6]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 146, 1, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 146, 2, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 153, 1, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 153, 2, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 154, 0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, 6]
#wait for mode
#[0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, 1]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 129, 0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 131, 0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 122, 0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 0]
#go
#[0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 122, 0, 0, 0, 0, 0]
#[0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 0]
#go

