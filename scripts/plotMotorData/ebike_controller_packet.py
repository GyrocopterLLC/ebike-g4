'''

Creates and upacks packets using the ebike-controller format.
Packets are constructed as follows:

    0->1: Start of Packet
       2: PacketID
       3: nPacketID
    4->5: length of data field (n)
  6->6+n: data
7+n-10+n: CRC32 on bytes 0->6+n

'''

import zlib

class Packet:
    SOP1 = 0x9A
    SOP2 = 0xCC
    def __init__(self, packetID, data):
        self.PacketID = packetID
        self.Data = data

def crc32_padded(input_bytes):
    packet_for_crc = input_bytes
    while((len(packet_for_crc) % 4) != 0):
        packet_for_crc = packet_for_crc + bytes([0])

    crc32 = zlib.crc32(packet_for_crc)
    return crc32

def ebike_unpack(raw_bytes):
    # Search for the start string
    sop_loc = raw_bytes.find(bytes([Packet.SOP1,Packet.SOP2]))
    if(sop_loc < 0):
        # No packets found
        return (Packet(0,0), -1, -1)
    if(sop_loc+6 > len(raw_bytes)):
        # Raw data length too short, but packet might still be coming
        return (Packet(0,0), sop_loc, -1)

    # Get packet ID and nPacket ID
    packetID = raw_bytes[sop_loc + 2]
    nPacketID = raw_bytes[sop_loc + 3]
    if(not (packetID == ((~nPacketID)&255))):
         # Packet ID failed, could have been false SOP
         # Try remainder of data with some recursion
        (pkt, sop_loc, packet_length) = ebike_unpack(raw_bytes[sop_loc+2:])
        if(sop_loc >= 0 and packet_length > 0):
            return (pkt, sop_loc, packet_length)
        else:
            return (Packet(0,0), -1, -1)
    
    data_length = (raw_bytes[sop_loc+4] << 8) + raw_bytes[sop_loc+5]
    if(sop_loc+6+data_length+4 > len(raw_bytes)):
        # Raw data length too short, but packet might still be coming
        return (Packet(0,0), sop_loc, -1)
    
    data = raw_bytes[sop_loc+6:sop_loc+6+data_length]
    remote_crc = raw_bytes[sop_loc+6+data_length:sop_loc+10+data_length]
    local_crc = crc32_padded(raw_bytes[sop_loc:sop_loc+6+data_length]).to_bytes(4,byteorder='big')
    if(remote_crc != local_crc):
        # CRC failed. Check the rest of the data for SOP and packet
        (pkt, sop_loc, packet_length) = ebike_unpack(raw_bytes[sop_loc+2:])
        if(sop_loc >= 0 and packet_length > 0):
            return (pkt, sop_loc, packet_length)
        else:
            return (Packet(0,0), -1, -1)
    # If we get here, everything was succesful!
    return (Packet(packetID, data), sop_loc, 10+data_length)

def ebike_pack(packetID, data):
    packet_bytes = bytearray([Packet.SOP1,Packet.SOP2])
    
    packet_bytes+=bytes([packetID&255, ((~packetID)&255)])
    
    data_length = len(data)
    packet_bytes+=bytes([((data_length&0xFF00) >> 8), (data_length&0x00FF)])

    packet_bytes+=data

    mycrc = crc32_padded(packet_bytes).to_bytes(4,byteorder='big')
    packet_bytes+=mycrc
    return packet_bytes
    