from pymavlink import mavutil
import datetime
import json
import os
import numpy as np
import pymap3d as pm
import time

# https://github.com/ArduPilot/pymavlink/blob/master/tools/mavtelemetry_datarates.py
# ESTIMATOR_STATUS -> pos_horiz_accuracy
#### KINEMATIC MODEL FUNCTIONS ###################

def predict_next_position(origin, current):
    
    x, y, z = geodesic_distances(current['lat'], current['lon'], current['alt'], origin[0], origin[1], origin[2])
    
    x = moving_average(x, 4)
    y = moving_average(y, 4)
    z = moving_average(z, 4)
    # Since in the kinematic model dt is calculated which in our case is impossible since we do not know the timestamp of the incoming message
    # I will be assuming a dt of 0.333 since that seems to be an average
    dt = 0.333
    x_pred = x + current['vx'] * dt
    y_pred = y + current['yx'] * dt
    z_pred = z + current['zx'] * dt
    # Finally send this to gimbel
    #TODO : Send data to gimbel

def geodesic_distances(lat1, lon1, alt1, lat2, lon2, alt2):
    # In mavlink, east is y, north is x, down is z (z flipped upwards in preprocessing)
    y, x, z = pm.geodetic2enu(lat1, lon1, alt1, lat2, lon2, alt2)
    return x, y, z

def moving_average(x, n=3):
    return np.convolve(x, np.ones(n), 'same') / n

#####################################################
def connect(connection_string, baud=57600):
    # Connect to the vehicle
    mav = mavutil.mavlink_connection(
        connection_string,
        baud=baud,
    )
    print("Waiting for heartbeat")
    mav.wait_heartbeat()
    return mav

def request_data_stream(mav, stream_id=mavutil.mavlink.MAV_DATA_STREAM_ALL, stream_rate=200):
    mav.mav.request_data_stream_send(
        mav.target_system,
        mav.target_component,
        stream_id,
        stream_rate,
        1,  # 1 to start sending, 0 to stop
    )

def record_data(mav):
    unique_mavpackettypes = set()
    messages = []
    try:
        i = 0
        global_pos_counter = 0
        start_time = time.time()
        while True:
            # Wait for a new message
            msg = mav.recv_match(blocking=True).to_dict()
            if msg['mavpackettype'] == 'GLOBAL_POSITION_INT':

                if global_pos_counter == 0:
                    #Origin is same as origin df in kinematic model repo. It is used to calculate x,y,z based on lat,lon,alt
                    origin = [msg['lat'], msg['lon'], msg['alt']]
                    global_pos_counter += 1
                    #Don't predict just shoot for origin
                else:
                    predict_next_position(origin, msg)
            # Probably the prediction would go here
            messages.append(msg)
            unique_mavpackettypes.add(msg.get('mavpackettype'))
            i += 1
            if i % 100 == 0:
                end_time = time.time()
                elapsed_time = end_time - start_time
                average_hz = 100 / elapsed_time if elapsed_time > 0 else 0
                print(f"{i} messages received, Average Hz: {average_hz:.2f}, Number of unique mavpackettypes: {len(unique_mavpackettypes)}")
                start_time = time.time()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        dt_string = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        json_data = json.dumps(messages)
        if not os.path.exists("./data/"):
            os.makedirs("./data/")
        with open(f"./data/{dt_string}.json", "w") as f:
            f.write(json_data)
        mav.close()

if __name__ == "__main__":
    connection_string = "/dev/cu.usbserial-D30F0LHK"
    mav = connect(connection_string)
    request_data_stream(mav, stream_rate=3)
    request_data_stream(mav, stream_id=mavutil.mavlink.MAV_DATA_STREAM_POSITION, stream_rate=50) # position
    request_data_stream(mav, stream_id=mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, stream_rate=50) # attitude

    record_data(mav)
    # lower the stream rate
    request_data_stream(mav, stream_rate=3)
