'''
    Read h264 raw frames and save it to a file
'''
import socket


socket_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tello_address = ('192.168.10.1', 8889)
local_address = ('', 11111)
socket_video.bind(local_address)

socket_video.sendto(b'command', tello_address)
print ('sent: command')
socket_video.sendto(b'streamon', tello_address)
print ('sent: streamon')

video_sample_file = open('tello.h264', 'wb')

packet_data = ""
num_frames = 0
#start_time = 0
while True:
    try:
        res_string, ip = socket_video.recvfrom(2048)
        #print('Bytes just received {0}'.format(len(res_string)))
        packet_data += res_string
        # if num_packets == 0:
        #     start_time = time.time()
        # num_packets = num_packets + 1
        # if num_packets == 200:
        #     print('Took {} ms to receive 200 packets'.format(str((time.time() - start_time) * 1000))) 
        #     num_packets = 0
        # end of frame
        if len(res_string) != 1460:
            #print('Bytes to decode {0}'.format(len(packet_data)))
            #print(' '.join([str(x) for x in map(ord, packet_data[:14])]))
            #for frame in self._h264_decode(packet_data):
            #    self.frame = frame
            ## Instead of decoding the packet, send it through websockets
            #VideoTransportProtocol.broadcast_message(packet_data)

            ## Send the video data through udp
            #self.video_transport_udp.send(packet_data)

            video_sample_file.write(packet_data)
            num_frames = num_frames + 1
            if num_frames == 800:
                break
            packet_data = ""

    except socket.error as exc:
        print ("Caught exception socket.error : %s" % exc)

video_sample_file.close()