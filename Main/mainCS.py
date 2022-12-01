import socket


# Addresses and ports
localAddress = "127.0.0.1"
buffersize = 9

class subsys:
    def __init__(self, CLIENT_PORT, SERVER_PORT):
        self.CLIENT_PORT = CLIENT_PORT
        self.SERVER_PORT = SERVER_PORT
        self.s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.s.bind((localAddress, self.CLIENT_PORT))

ur10 = subsys(20001, 20002)
rail = subsys(20003, 20004)
camera = subsys(20005, 20006)

def extractBytes(integer):
    firstArray = divmod(integer, 0x100)
    secondArray = divmod(firstArray[0], 0x100)
    thirdArray = divmod(secondArray[0], 0x100)
    output = [thirdArray[0], thirdArray[1], secondArray[1], firstArray[1]]
    return output

def combineBytes(sign, bytes):
    output = bytes[0]*256**3 + bytes[1]*256**2 + bytes[2]*256 + bytes[3]
    if sign == 1:
        output = output*(-1)
    return output

def communicateUDP(sub_system, object, subindex=0, rw=0, information=0, nr_of_following_messages=0):

    # Format data
    sign = 0
    if information < 0:
        sign = 1
        information = abs(information)
    information = extractBytes(information)
    package = [object, subindex, rw, sign, information[0], information[1], information[2], information[3], nr_of_following_messages]
    package_array = bytes(package)

    # Send data
    sub_system.s.sendto(package_array, (localAddress, sub_system.SERVER_PORT))

    # Recieve data
    if rw == 0:
        recieved = sub_system.s.recvfrom(buffersize)
        recieved = list(recieved[0])
        recieved = combineBytes(recieved[3], recieved[4:8])
        return recieved

communicateUDP(ur10, 1, 1, rw=1, information=100500, nr_of_following_messages=2)
communicateUDP(ur10, 1, 2, rw=1, information=100400, nr_of_following_messages=1)
communicateUDP(ur10, 1, 3, rw=1, information=100300, nr_of_following_messages=0)

print(communicateUDP(ur10, 1, 1))
print(communicateUDP(ur10, 1, 2))
print(communicateUDP(ur10, 1, 3))

communicateUDP(ur10, 1, 3, rw=1, information=1500)

print(communicateUDP(ur10, 1, 3))

