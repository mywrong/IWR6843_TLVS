from radar import TI
import numpy as np
import time

class Detected_Points:

    def data_stream_iterator(self,cli_loc='COM4',data_loc='COM3',total_frames=300):
        
        MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'
        ti=TI(cli_loc=cli_loc,data_loc=data_loc)
        nframe=0
        interval=0.05
        data=b''
        warn=0
        while nframe<total_frames:

            time.sleep(interval)
            byte_buffer=ti._read_buffer()
            
            if(len(byte_buffer)==0):
                warn+=1
            else:
                warn=0
            if(warn>10):#连续10次空读取则退出
                print("Wrong")
                break
        
            data+=byte_buffer
        
            try:
                idx1 = data.index(MAGIC_WORD)   
                idx2=data.index(MAGIC_WORD,idx1+1)

            except:
                continue

            datatmp=data[idx1:idx2]
            data=data[idx2:]
            points=ti._process_detected_points(byte_buffer)
            ret=points[:,:3]

            yield ret
            nframe+=1

        ti.close()