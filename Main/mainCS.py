import socket


# Addresses and ports
localAddress = "127.0.0.1"
buffersize = 8

class subsys:
    def __init__(self, CLIENT_PORT, SERVER_PORT, s):
        self.CLIENT_PORT = CLIENT_PORT
        self.SERVER_PORT = SERVER_PORT
        self.s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        s.bind((localAddress, self.CLIENT_PORT))

ur10 = subsys(20001, 20002)
rail = subsys(20003, 20004)
camera = subsys(20005, 20006)

# Sockets and bindings
# ur10 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# ur10.bind((localAddress, ur10PORTS.CLIENT_PORT))
#
# rail = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# rail.bind((localAddress, railPORTS.CLIENT_PORT))
#
# camera = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# camera.bind((localAddress, cameraPORTS.CLIENT_PORT))

def extractBytes(integer):
    firstArray = divmod(integer, 0x100)
    secondArray = divmod(firstArray[0], 0x100)
    thirdArray = divmod(secondArray[0], 0x100)
    output = [thirdArray[0], thirdArray[1], secondArray[1], firstArray[1]]
    return output

def communicateUDP(sub_system, object, subindex=0, rw=0, information=0):

    # Format data
    sign = 0
    if information > 0:
        sign = 1
        information = abs(information)
    information = extractBytes(information)
    package = [object, subindex, rw, sign, information[0], information[1], information[2], information[3]]
    package_array = bytes(package)

    # Send data
    sub_system.sendto(package_array, (localAddress, sub_system.SERVER_PORT))

    # Recieve data
    recieved = sub_system.recvfrom(buffersize)
    recieved = list(recieved)
    return recieved


