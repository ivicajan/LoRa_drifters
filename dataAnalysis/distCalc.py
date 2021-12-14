import math
from datetime import datetime

# same as TinyGPS lib calc
def getDistance(lat1, lon1, lat2, lon2):
    delta = math.radians(lon1-lon2)
    sdlong = math.sin(delta)
    cdlong = math.cos(delta)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    slat1 = math.sin(lat1)
    clat1 = math.cos(lat1)
    slat2 = math.sin(lat2)
    clat2 = math.cos(lat2)
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong)
    delta = delta ** 2
    delta += (clat2 * sdlong) ** 2
    delta = math.sqrt(delta)
    denom = (slat1 * slat2) + (clat1 * clat2 * cdlong)
    delta = math.atan2(delta, denom)
    return delta * 6372795

def main():
    d01_lat1 = -31.99754833 # start
    d01_lon1 = 115.8287333
    d01_lat2 = -31.99984083 # end
    d01_lon2 = 115.8286672

    d02_lat1 = -31.99753967 # start
    d02_lon1 = 115.8287335
    d02_lat2 = -32.00024733 # end
    d02_lon2 = 115.8275255

    start_time = datetime(2021,11, 8, 1, 56, 0)           # "1:56:00" +8 -> 9:56 AM
    end_time = datetime(2021,11, 8, 2, 48, 0)             # "2:48:00" +8 -> 10:48 AM

    time_delta_s = (end_time - start_time).total_seconds()
    print('Total drift time')
    print(time_delta_s, 's\n')

    d01_dist = getDistance(d01_lat1, d01_lon1, d01_lat2, d01_lon2)
    d02_dist = getDistance(d02_lat1, d02_lon1, d02_lat2, d02_lon2)
    print('Total distance')
    print('D01: ', d01_dist, 'm')
    print('D02: ', d02_dist, 'm')
    d01_velocity = d01_dist/time_delta_s
    d02_velocity = d02_dist/time_delta_s
    print('Velocity')
    print('D01: ', d01_velocity, 'm/s')
    print('D02: ', d02_velocity, 'm/s')

if __name__=='__main__':
    main()